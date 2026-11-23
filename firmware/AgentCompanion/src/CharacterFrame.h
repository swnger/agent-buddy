#pragma once
#include "CharacterMotion.h"
#include "CharacterPack.h"
#include "FullFrameRenderer.h"
#include "SpriteRenderer.h"

namespace copilot {
// One step of the character, shared by the device (AgentCompanion.ino), the
// desktop engine (desktop/engine) and the host preview (tools/character_preview.cpp),
// so all three apply requests, keep time and pick a renderer the same way.

// A mode change as it is queued. A tap (or a click on the desktop character) is a
// Surprise like any other: after the spring it resumes the agent's Working or
// Needs attention state, or Idle when there is none.
struct ModeRequest {
  CharacterMode mode = CharacterMode::Idle;
};

// Applies a request to the motion. False when the motion reports an error.
bool applyModeRequest(CharacterMotion& motion, const ModeRequest& request);

// Advances the motion by wall-clock seconds, at the pack's motion speed. The
// device passes the header it read at start: an install unmaps the pack while
// the render task may still be stepping.
void stepCharacterMotion(CharacterMotion& motion, double seconds, const PackHeader& pack);

inline SpritePose framePose(const CharacterState& state, const PackHeader& pack) {
  SpritePose pose = state.pose;
  pose.index = walkIndex(pack, pose.direction, pose.index, state.effectSeconds);
  return pose;
}

// Roll the real motion engine forward on a copy. Spread refills over several refreshes
// so a new command or a change of cadence cannot spend a frame on a full prediction.
class PoseForecast {
 public:
  static constexpr unsigned kHorizon = 120;
  void advance(const CharacterMotion& motion, const PackHeader& pack, double dt) {
    const auto state = motion.state();
    const auto actual = framePose(state, pack);
    if (!size_ || event_ != state.eventId || at(0).direction != actual.direction || at(0).index != actual.index) {
      preview_ = motion;
      event_ = state.eventId;
      size_ = head_ = 0;
    } else {
      head_ = (head_ + 1) % kHorizon;
      --size_;
    }
    for (int step = 0; step < 3 && size_ < kHorizon; ++step) {
      preview_.update(dt * pack.motionSpeed);
      poses_[(head_ + size_++) % kHorizon] = framePose(preview_.state(), pack);
    }
  }
  unsigned size() const { return size_; }
  SpritePose at(unsigned i) const { return poses_[(head_ + i) % kHorizon]; }

 private:
  CharacterMotion preview_{1};
  SpritePose poses_[kHorizon];
  unsigned head_ = 0, size_ = 0;
  uint32_t event_ = 0;
};

// Draws the pose with the renderer the loaded pack's layout needs: full-frame
// packs carry whole images, base-patch packs a base and eye patches. Effects
// are drawn separately, after this.
class CharacterSprite {
 public:
  CharacterSprite(SpriteRenderer* patch, FullFrameRenderer* fullFrame)
      : patch_(patch), fullFrame_(fullFrame) {}
  bool render(const CharacterState& state, uint16_t* frame);
  void invalidate();
  // True when the installed pack uses the full-frame layout.
  static bool fullFrame();
  const char* error() const;

 private:
  SpriteRenderer* patch_;
  FullFrameRenderer* fullFrame_;
};
}
