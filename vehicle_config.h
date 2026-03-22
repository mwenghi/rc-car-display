#pragma once
#include <stdint.h>

// ─── Vehicle Configuration Menu (dynamic items from I2C master) ─────────────

#define VCONF_MAX_ITEMS    16
#define VCONF_MAX_OPTIONS   6
#define VCONF_LABEL_LEN    11  // 10 chars + null

enum VConfItemType : uint8_t {
  VCONF_TOGGLE  = 0,
  VCONF_NUMBER  = 1,
  VCONF_OPTIONS = 2,
  VCONF_ACTION  = 3,
  VCONF_SUBMENU = 4
};

struct VConfItem {
  bool     defined;
  uint8_t  id;            // 0-15
  VConfItemType type;
  uint8_t  flags;         // bit0=readonly, bit1=hidden
  char     name[VCONF_LABEL_LEN];
  int16_t  value;         // toggle:0/1, number:val, options:idx
  // Number constraints
  int16_t  numMin;
  int16_t  numMax;
  int16_t  numStep;
  // Options
  uint8_t  optionCount;
  char     optionLabels[VCONF_MAX_OPTIONS][VCONF_LABEL_LEN];
  // Grouping
  uint8_t  groupId;       // 0xFF = root, else id of parent submenu
};

// ─── Event queue (ESP32 → MEGA feedback) ────────────────────────────────────

#define VCONF_EVENT_QUEUE_SIZE 8

struct VConfEvent {
  uint8_t itemId;
  int16_t value;
};

struct VConfState {
  VConfItem items[VCONF_MAX_ITEMS];
  uint8_t   itemCount;     // number of defined items

  // Circular event queue
  volatile VConfEvent eventQueue[VCONF_EVENT_QUEUE_SIZE];
  volatile uint8_t eventHead;  // written from loop()
  volatile uint8_t eventTail;  // read from ISR (onRequest)

  // Number editing state
  bool     editing;
  uint8_t  editSlot;       // index in items[]
  int16_t  editValue;
};

VConfState vconf = {0};

// ─── Helpers ────────────────────────────────────────────────────────────────

// Count visible root items (defined, not hidden, groupId==0xFF)
int vconfRootCount() {
  int n = 0;
  for (int i = 0; i < VCONF_MAX_ITEMS; i++)
    if (vconf.items[i].defined && !(vconf.items[i].flags & 0x02) && vconf.items[i].groupId == 0xFF)
      n++;
  return n;
}

// Count visible children of a group
int vconfChildCount(uint8_t groupId) {
  int n = 0;
  for (int i = 0; i < VCONF_MAX_ITEMS; i++)
    if (vconf.items[i].defined && !(vconf.items[i].flags & 0x02) && vconf.items[i].groupId == groupId)
      n++;
  return n;
}

// Get nth visible root item (returns nullptr if out of range)
VConfItem* vconfRootItem(int idx) {
  int n = 0;
  for (int i = 0; i < VCONF_MAX_ITEMS; i++) {
    if (vconf.items[i].defined && !(vconf.items[i].flags & 0x02) && vconf.items[i].groupId == 0xFF) {
      if (n == idx) return &vconf.items[i];
      n++;
    }
  }
  return nullptr;
}

// Get nth visible child of a group
VConfItem* vconfChildItem(uint8_t groupId, int idx) {
  int n = 0;
  for (int i = 0; i < VCONF_MAX_ITEMS; i++) {
    if (vconf.items[i].defined && !(vconf.items[i].flags & 0x02) && vconf.items[i].groupId == groupId) {
      if (n == idx) return &vconf.items[i];
      n++;
    }
  }
  return nullptr;
}

// Find item by id
VConfItem* vconfFindItem(uint8_t id) {
  for (int i = 0; i < VCONF_MAX_ITEMS; i++)
    if (vconf.items[i].defined && vconf.items[i].id == id)
      return &vconf.items[i];
  return nullptr;
}

// ─── Event queue ────────────────────────────────────────────────────────────

void vconfEnqueueEvent(uint8_t itemId, int16_t value) {
  uint8_t next = (vconf.eventHead + 1) % VCONF_EVENT_QUEUE_SIZE;
  if (next != vconf.eventTail) {  // not full
    vconf.eventQueue[vconf.eventHead].itemId = itemId;
    vconf.eventQueue[vconf.eventHead].value = value;
    vconf.eventHead = next;
  }
}

// Dequeue one event into 4-byte response buffer (called from ISR)
void vconfDequeueEvent(uint8_t resp[4]) {
  if (vconf.eventTail != vconf.eventHead) {
    volatile VConfEvent& e = vconf.eventQueue[vconf.eventTail];
    resp[0] = e.itemId;
    resp[1] = (uint8_t)(e.value & 0xFF);
    resp[2] = (uint8_t)((e.value >> 8) & 0xFF);
    resp[3] = resp[0] ^ resp[1] ^ resp[2];
    vconf.eventTail = (vconf.eventTail + 1) % VCONF_EVENT_QUEUE_SIZE;
  } else {
    resp[0] = 0xFF;  // no event
    resp[1] = 0;
    resp[2] = 0;
    resp[3] = 0xFF;
  }
}

// ─── I2C message parsers ────────────────────────────────────────────────────

// 0x20: Define item
// [0] id  [1] type  [2] flags  [3..12] name(10)
// then type-specific:
//   toggle:  [13] value(u8)
//   number:  [13..14] value(i16) [15..16] min(i16) [17..18] max(i16) [19..20] step(i16)
//   options: [13] current_idx  [14] option_count
//   action:  (nothing)
//   submenu: [13] parent_id (for the submenu itself, usually 0xFF)
void vconfParseDefine(const uint8_t* p, uint8_t len) {
  if (len < 13) return;
  uint8_t id = p[0];
  if (id >= VCONF_MAX_ITEMS) return;

  VConfItem& item = vconf.items[id];
  bool wasNew = !item.defined;
  item.defined = true;
  item.id = id;
  item.type = (VConfItemType)p[1];
  item.flags = p[2];
  memset(item.name, 0, VCONF_LABEL_LEN);
  memcpy(item.name, &p[3], min((int)len - 3, 10));
  item.name[10] = '\0';
  item.groupId = 0xFF;  // default root

  switch (item.type) {
    case VCONF_TOGGLE:
      if (len >= 14) item.value = p[13];
      break;
    case VCONF_NUMBER:
      if (len >= 21) {
        memcpy(&item.value,   &p[13], 2);
        memcpy(&item.numMin,  &p[15], 2);
        memcpy(&item.numMax,  &p[17], 2);
        memcpy(&item.numStep, &p[19], 2);
      }
      break;
    case VCONF_OPTIONS:
      if (len >= 15) {
        item.value = p[13];
        item.optionCount = min(p[14], (uint8_t)VCONF_MAX_OPTIONS);
      }
      break;
    case VCONF_ACTION:
      item.value = 0;
      break;
    case VCONF_SUBMENU:
      if (len >= 14) item.groupId = p[13];
      else item.groupId = 0xFF;
      item.value = 0;
      break;
  }

  if (wasNew) vconf.itemCount++;
  Serial.printf("VConf: defined item %d '%s' type=%d\n", id, item.name, item.type);
}

// 0x21: Option label
// [0] item_id  [1] option_idx  [2..11] label(10)
void vconfParseOptionLabel(const uint8_t* p, uint8_t len) {
  if (len < 3) return;
  VConfItem* item = vconfFindItem(p[0]);
  if (!item || item->type != VCONF_OPTIONS) return;
  uint8_t oidx = p[1];
  if (oidx >= VCONF_MAX_OPTIONS) return;
  memset(item->optionLabels[oidx], 0, VCONF_LABEL_LEN);
  memcpy(item->optionLabels[oidx], &p[2], min((int)len - 2, 10));
  item->optionLabels[oidx][10] = '\0';
}

// 0x22: Update value
// [0] item_id  [1..2] value(i16)
void vconfParseUpdateValue(const uint8_t* p, uint8_t len) {
  if (len < 3) return;
  VConfItem* item = vconfFindItem(p[0]);
  if (!item) return;
  int16_t val;
  memcpy(&val, &p[1], 2);
  item->value = val;
  Serial.printf("VConf: update item %d = %d\n", item->id, val);
}

// 0x23: Clear
// [0] item_id (0xFF = all)
void vconfParseClear(const uint8_t* p, uint8_t len) {
  if (len < 1) return;
  if (p[0] == 0xFF) {
    memset(&vconf.items, 0, sizeof(vconf.items));
    vconf.itemCount = 0;
    Serial.println("VConf: cleared all");
  } else {
    VConfItem* item = vconfFindItem(p[0]);
    if (item) {
      item->defined = false;
      vconf.itemCount--;
      Serial.printf("VConf: cleared item %d\n", p[0]);
    }
  }
}

// 0x24: Set group (assign item to a submenu parent)
// [0] item_id  [1] group_id
void vconfParseSetGroup(const uint8_t* p, uint8_t len) {
  if (len < 2) return;
  VConfItem* item = vconfFindItem(p[0]);
  if (item) {
    item->groupId = p[1];
    Serial.printf("VConf: item %d -> group %d\n", p[0], p[1]);
  }
}

// ─── Item interaction (called from menu system) ─────────────────────────────

// Enter/confirm on an item — returns true if event was generated
bool vconfItemEnter(VConfItem* item) {
  if (!item || (item->flags & 0x01)) return false;  // readonly

  switch (item->type) {
    case VCONF_TOGGLE:
      item->value = item->value ? 0 : 1;
      vconfEnqueueEvent(item->id, item->value);
      Serial.printf("VConf: toggle %d = %d\n", item->id, item->value);
      return true;

    case VCONF_NUMBER:
      if (!vconf.editing) {
        // Enter edit mode
        vconf.editing = true;
        vconf.editSlot = item->id;
        vconf.editValue = item->value;
        return false;
      } else {
        // Confirm edit
        item->value = vconf.editValue;
        vconf.editing = false;
        vconfEnqueueEvent(item->id, item->value);
        Serial.printf("VConf: number %d = %d\n", item->id, item->value);
        return true;
      }

    case VCONF_OPTIONS:
      if (item->optionCount > 0) {
        item->value = (item->value + 1) % item->optionCount;
        vconfEnqueueEvent(item->id, item->value);
        Serial.printf("VConf: option %d = %d\n", item->id, item->value);
        return true;
      }
      return false;

    case VCONF_ACTION:
      vconfEnqueueEvent(item->id, 1);
      Serial.printf("VConf: action %d triggered\n", item->id);
      return true;

    case VCONF_SUBMENU:
      // Handled by menu navigation (enter L3)
      return false;
  }
  return false;
}

// Adjust number value during editing (delta = +1 or -1 step)
void vconfItemAdjust(int8_t direction) {
  if (!vconf.editing) return;
  VConfItem* item = vconfFindItem(vconf.editSlot);
  if (!item) { vconf.editing = false; return; }

  int16_t step = item->numStep ? item->numStep : 1;
  vconf.editValue += direction * step;
  if (vconf.editValue < item->numMin) vconf.editValue = item->numMin;
  if (vconf.editValue > item->numMax) vconf.editValue = item->numMax;
}

// Cancel number editing
void vconfCancelEdit() {
  vconf.editing = false;
}
