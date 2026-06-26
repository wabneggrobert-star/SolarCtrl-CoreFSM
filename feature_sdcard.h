#pragma once
namespace SDCard {
  bool begin();
  bool ensureReady();
  bool exists(const char* path);
}
