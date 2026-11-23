#include "CharacterMotion.h"
#include "CharacterModel.h"
#include "MotionMath.h"
#include <algorithm>
#include <cmath>

namespace copilot {
namespace {
bool latched(CharacterMode mode) {
  return mode == CharacterMode::Working || mode == CharacterMode::Attention;
}
bool preservesExpression(CharacterMode mode) {
  return mode == CharacterMode::Surprise || mode == CharacterMode::Complete
      || mode == CharacterMode::Sleep;
}
bool stable(CharacterMode mode) {
  return latched(mode) || mode == CharacterMode::Idle || mode == CharacterMode::Sleep;
}
}

CharacterMotion::CharacterMotion(uint32_t seed)
    : CharacterMotion(seed, kSpriteSteps == 24 && kSpriteBlinkLevels == 5 ? kSpriteDirections : 0) {}

CharacterMotion::CharacterMotion(uint32_t seed, uint8_t availableDirections)
    : idle_(seed), available_(availableDirections), random_(seed ? seed : 0x6d2b79f5u) {
  if (available_ < 8) error_ = "Character assets require eight idle tracks, 24 poses and five blink levels.";
}

double CharacterMotion::range(double low, double high) {
  return low + double(xorshift32(random_)) / 4294967296.0 * (high - low);
}

bool CharacterMotion::setMode(CharacterMode mode) {
  if (static_cast<uint8_t>(mode) > static_cast<uint8_t>(CharacterMode::Sleep)) {
    error_ = "Unknown character mode.";
    return false;
  }
  if (available_ < 8
      || (mode != CharacterMode::Idle && mode != CharacterMode::Sleep && available_ < 13)) {
    error_ = "Expression assets unavailable: export all 13 character tracks first.";
    return false;
  }
  error_ = nullptr;
  if (stable(mode) && next_ == mode) return true;
  if (mode != CharacterMode::Surprise) persistent_ = latched(mode) ? mode : CharacterMode::Idle;
  if (mode_ == CharacterMode::Surprise) repeatSurprise_ = mode == CharacterMode::Surprise;
  next_ = mode;
  ++eventId_;
  effectSeconds_ = 0;
  idleSeconds_ = 0;
  if (mode_ == CharacterMode::Sleep && mode != CharacterMode::Sleep) {
    sleepExiting_ = true;
    sleepTransitionSeconds_ = 0;
    if (!idle_.returnToCenter(1.2)) return false;
    return true;
  }
  if (mode_ == CharacterMode::Idle) {
    idle_.returnToCenter(mode == CharacterMode::Surprise ? .35 : .65);
  } else {
    returnExpression();
  }
  return true;
}

void CharacterMotion::surprise() { setMode(CharacterMode::Surprise); }

bool CharacterMotion::requestIdleDirection(int direction) {
  if (next_ != CharacterMode::Idle) {
    error_ = "Idle direction preview requires Idle mode.";
    return false;
  }
  if (!idle_.returnToCenter(.25)) {
    error_ = idle_.error();
    return false;
  }
  if (!idle_.request(direction)) {
    error_ = idle_.error();
    return false;
  }
  idle_.setAutomatic(true);
  error_ = nullptr;
  return true;
}

CharacterState CharacterMotion::state() const {
  SpritePose pose = mode_ == CharacterMode::Idle ? idle_.pose() : pose_;
  if (mode_ == CharacterMode::Sleep) {
    pose = idle_.pose();
    if (sleepExiting_) {
      pose.blinkLevel = pose.index ? 4 : static_cast<uint8_t>(std::max(
          0, 4 - static_cast<int>(sleepTransitionSeconds_ / .035)));
    } else if (sleepTransitionSeconds_ < .12) {
      pose.blinkLevel = static_cast<uint8_t>(std::min(
          4, static_cast<int>(sleepTransitionSeconds_ / .03)));
    } else {
      const double sleepy = std::fmod(sleepSeconds_, 10.0);
      if (sleepy < 4) {
        double openness;
        if (sleepy < 1.5) openness = quintic(sleepy / 1.5);
        else if (sleepy < 2.5) openness = 1;
        else openness = quintic((4 - sleepy) / 1.5);
        if (openness > 0) {
          pose.blinkLevel = 3;
          pose.blinkBlend = static_cast<uint8_t>(
              std::max(1l, std::lround(255 * openness)));
        } else {
          pose.blinkLevel = 4;
        }
      } else {
        pose.blinkLevel = 4;
      }
    }
  } else {
    pose.blinkLevel = idle_.pose().blinkLevel;
  }
  return {pose, mode_, next_, static_cast<float>(effectSeconds_), eventId_};
}

size_t CharacterMotion::peekUpcoming(SpritePose* out, size_t maxCount) const {
  if (!out || maxCount == 0 || available_ < 8) return 0;
  size_t count = 0;
  auto appendRange = [&](uint8_t dir, int firstIdx, int lastIdx) {
    if (dir >= available_ || dir >= kSpriteDirections || count >= maxCount) return;
    const int maxIdx = static_cast<int>(kSpriteTrackSteps[dir]) - 1;
    if (firstIdx < 0 || firstIdx > maxIdx || lastIdx < 0 || lastIdx > maxIdx) return;
    const int step = lastIdx >= firstIdx ? 1 : -1;
    for (int idx = firstIdx; count < maxCount; idx += step) {
      out[count++] = {dir, static_cast<uint8_t>(idx), 0, 0};
      if (idx == lastIdx) break;
    }
  };
  auto appendModeEntry = [&](CharacterMode targetMode) {
    if (targetMode == CharacterMode::Idle || targetMode == CharacterMode::Sleep || available_ < 13)
      return;
    const uint8_t dir = static_cast<uint8_t>(7 + static_cast<uint8_t>(targetMode));
    appendRange(dir, 1, static_cast<int>(kSpriteTrackSteps[dir]) - 1);
  };

  if (mode_ == CharacterMode::Idle || mode_ == CharacterMode::Sleep) {
    const SpritePose cur = idle_.pose();
    const auto phase = idle_.phase();
    if (phase == SpriteMotion::Phase::Out) {
      if (cur.index < idle_.target()) appendRange(cur.direction, cur.index + 1, idle_.target());
    } else if (phase == SpriteMotion::Phase::Return || phase == SpriteMotion::Phase::Endpoint) {
      if (cur.index > 1) appendRange(cur.direction, cur.index - 1, 1);
    } else if (mode_ == CharacterMode::Idle && next_ == mode_) {
      // The center dwell gives the decoder a head start before the first moving frame.
      uint8_t direction, depth;
      if (idle_.peekNextTurn(direction, depth)) appendRange(direction, 1, depth);
    }
    if (next_ != mode_) appendModeEntry(next_);
    return count;
  }

  if (phase_ != Phase::Hold && phase_ != Phase::WorkLookHold && pose_.index != to_) {
    const int step = to_ > from_ ? 1 : -1;
    appendRange(pose_.direction, static_cast<int>(pose_.index) + step, to_);
  }

  if (phase_ == Phase::Returning) {
    appendModeEntry(next_);
  } else if (phase_ == Phase::SurpriseReaction) {
    const CharacterMode after = (next_ == CharacterMode::Surprise && !repeatSurprise_) ? persistent_ : next_;
    appendModeEntry(after);
  } else if (next_ != mode_) {
    if (pose_.index > 1 && (phase_ == Phase::Hold || phase_ == Phase::WorkLookHold))
      appendRange(pose_.direction, static_cast<int>(pose_.index) - 1, 1);
    appendModeEntry(next_);
  } else if (mode_ == CharacterMode::Attention) {
    if (phase_ == Phase::Hold && pose_.index > 1)
      appendRange(pose_.direction, static_cast<int>(pose_.index) - 1, 1);
    if (phase_ == Phase::Hold || phase_ == Phase::AttentionBack) {
      const uint8_t altDir = pose_.direction == 11 ? 12 : 11;
      appendRange(altDir, 1, static_cast<int>(kSpriteTrackSteps[altDir]) - 1);
    }
  } else if (mode_ == CharacterMode::Working) {
    if (pose_.direction == 9
        && (phase_ == Phase::Hold || phase_ == Phase::Micro || phase_ == Phase::WorkUnfocus)) {
      if (phase_ != Phase::WorkUnfocus && pose_.index > 1)
        appendRange(9, static_cast<int>(pose_.index) - 1, 1);
      uint32_t rng = random_;
      auto peekRange = [&rng](double low, double high) {
        return low + double(xorshift32(rng)) / 4294967296.0 * (high - low);
      };
      const uint8_t lookDir = static_cast<uint8_t>(peekRange(0, 2));
      const uint8_t lookTarget = static_cast<uint8_t>(peekRange(8, 14));
      appendRange(lookDir, 1, lookTarget);
    } else if (phase_ == Phase::WorkLookOut || phase_ == Phase::WorkLookHold
               || phase_ == Phase::WorkLookBack) {
      if (phase_ == Phase::WorkLookOut && to_ > 1)
        appendRange(pose_.direction, static_cast<int>(to_) - 1, 1);
      else if (phase_ == Phase::WorkLookHold && pose_.index > 1)
        appendRange(pose_.direction, static_cast<int>(pose_.index) - 1, 1);
      appendRange(9, 1, static_cast<int>(kSpriteTrackSteps[9]) - 1);
    }
  }
  return count;
}

void CharacterMotion::beginLeg(uint8_t to, double duration, Phase phase) {
  from_ = pose_.index;
  to_ = to;
  duration_ = duration;
  phase_ = phase;
  progress_ = 0;
  cachedIndex_ = -1;
}

void CharacterMotion::returnExpression() {
  // The spring track already settles forward to neutral; reversing would replay the impact.
  if (phase_ == Phase::SurpriseReaction) return;
  if (phase_ != Phase::Returning)
    beginLeg(0, next_ == CharacterMode::Surprise ? .35 : .65, Phase::Returning);
}

void CharacterMotion::enter(CharacterMode mode) {
  mode_ = mode;
  effectSeconds_ = 0;
  repeatSurprise_ = false;
  if (mode == CharacterMode::Idle) {
    idleSeconds_ = 0;
    idle_.setDuration(1.8);
    idle_.setAutomatic(true);
    return;
  }
  if (mode == CharacterMode::Sleep) {
    pose_ = idle_.pose();
    pose_.index = 0;
    sleepSeconds_ = 0;
    sleepTransitionSeconds_ = 0;
    sleepMotionWait_ = range(3.5, 6.5);
    sleepExiting_ = false;
    idle_.setAutomatic(false);
    idle_.setDuration(3.4);
    return;
  }
  pose_.direction = 7 + static_cast<uint8_t>(mode);
  pose_.index = 0;
  if (mode == CharacterMode::Working) workWait_ = range(4, 7);
  if (mode == CharacterMode::Surprise) {
    beginLeg(23, .75, Phase::SurpriseReaction);
  } else {
    beginLeg(23, mode == CharacterMode::Attention ? 1.4 : .95, Phase::Out);
  }
}

double CharacterMotion::edgeInterval() {
  if (cachedIndex_ == pose_.index) return cachedInterval_;
  const int steps = std::abs(int(to_) - from_);
  const int travelled = std::abs(int(pose_.index) - from_);
  // Physics is baked into equally spaced spring samples, not a second easing curve.
  double interval = phase_ == Phase::SurpriseReaction ? duration_ / steps
      : duration_ * (inverseQuintic(double(travelled + 1) / steps) - inverseQuintic(double(travelled) / steps));
  if (softEase_ && phase_ != Phase::SurpriseReaction) interval = std::min(interval, 0.115);
  cachedInterval_ = interval;
  cachedIndex_ = pose_.index;
  return cachedInterval_;
}

void CharacterMotion::advanceExpression(double dt) {
  if (phase_ == Phase::Returning && pose_.index == 0) {
    if (preservesExpression(next_) && idle_.blinkPhase() != SpriteMotion::BlinkPhase::Idle) return;
    enter(next_);
    return;
  }
  if (phase_ == Phase::SurpriseReaction && pose_.index == 23) {
    if (next_ == CharacterMode::Surprise && !repeatSurprise_) next_ = persistent_;
    enter(next_);  // Spring frame 23 and every track's frame 0 are identical.
    return;
  }
  if (phase_ == Phase::WorkUnfocus && pose_.index == 0) {
    pose_.direction = static_cast<uint8_t>(range(0, 2));
    const uint8_t target = static_cast<uint8_t>(range(8, 14));
    const double duration = range(1.5, 2.2);
    beginLeg(target, duration, Phase::WorkLookOut);
    return;
  }
  if (phase_ == Phase::WorkLookBack && pose_.index == 0) {
    pose_.direction = 9;
    workWait_ = range(4, 7);
    beginLeg(23, .95, Phase::Out);
    return;
  }
  if (phase_ == Phase::AttentionBack && pose_.index == 0) {
    pose_.direction = pose_.direction == 11 ? 12 : 11;
    beginLeg(23, 1.4, Phase::Out);
    return;
  }
  if (phase_ == Phase::WorkLookHold) {
    hold_ -= dt;
    if (hold_ <= 0) beginLeg(0, range(1.5, 2.2), Phase::WorkLookBack);
    return;
  }
  if (mode_ == CharacterMode::Working && pose_.direction == 9
      && (phase_ == Phase::Hold || phase_ == Phase::Micro)) {
    workWait_ -= dt;
    if (workWait_ <= 0) {
      beginLeg(0, 1.0, Phase::WorkUnfocus);
      return;
    }
  }
  if (phase_ == Phase::Hold) {
    hold_ -= dt;
    if (hold_ > 0) return;
    if (mode_ == CharacterMode::Attention) {
      beginLeg(0, 1.4, Phase::AttentionBack);
    } else if (latched(mode_)) {
      beginLeg(pose_.index == 23 ? 21 : 23, .45, Phase::Micro);
    } else {
      next_ = CharacterMode::Idle;
      returnExpression();
    }
    return;
  }
  const double interval = edgeInterval();
  progress_ += dt / interval;
  if (progress_ < 1) return;
  const double remainder = (progress_ - 1) * interval;
  pose_.index += to_ > from_ ? 1 : -1;
  progress_ = 0;
  if (pose_.index == to_) {
    if (phase_ == Phase::WorkLookOut) {
      phase_ = Phase::WorkLookHold;
      hold_ = range(.5, 1.2);
    } else if (phase_ != Phase::Returning && phase_ != Phase::WorkUnfocus
               && phase_ != Phase::WorkLookBack && phase_ != Phase::AttentionBack
               && phase_ != Phase::SurpriseReaction) {
      phase_ = Phase::Hold;
      hold_ = mode_ == CharacterMode::Attention ? range(2.2, 4.2)
          : latched(mode_) ? (pose_.index == 23 ? 1.4 : .9)
          : 1.1;
    }
  } else {
    progress_ = std::min(remainder / edgeInterval(), 1.0);
  }
}

void CharacterMotion::update(double dt) {
  if (!playing_ || available_ < 8 || !std::isfinite(dt) || dt <= 0) return;
  dt = std::min(dt, 1.0 / 30);
  idle_.setBlinks(!preservesExpression(mode_) && !preservesExpression(next_));
  effectSeconds_ = std::fmod(effectSeconds_ + dt, 12.0);
  if (mode_ == CharacterMode::Idle) {
    if (next_ != CharacterMode::Idle && idle_.pose().index == 0) {
      idle_.update(dt);
      if (preservesExpression(next_) && idle_.blinkPhase() != SpriteMotion::BlinkPhase::Idle) return;
      enter(next_);
      return;
    }
    // An explicit Idle can cancel an expression request while retracing an idle track.
    if (next_ == CharacterMode::Idle && idle_.pose().index == 0 && !idle_.automatic()) {
      idle_.setDuration(1.8);
      idle_.setAutomatic(true);
    }
    idle_.update(dt);
    if (next_ == CharacterMode::Idle) {
      idleSeconds_ += dt;
      if (idleSeconds_ >= kIdleBeforeSleepSeconds) setMode(CharacterMode::Sleep);
    }
  } else if (mode_ == CharacterMode::Sleep) {
    idle_.update(dt);
    if (sleepExiting_) {
      if (idle_.pose().index) return;
      sleepTransitionSeconds_ += dt;
      if (sleepTransitionSeconds_ >= .14) enter(next_);
      return;
    }
    sleepTransitionSeconds_ += dt;
    sleepSeconds_ += dt;
    sleepMotionWait_ -= dt;
    if (sleepMotionWait_ <= 0 && idle_.pose().index == 0
        && idle_.phase() == SpriteMotion::Phase::Center) {
      if (!idle_.request(range(0, 1) < .5 ? 0 : 1, range(.12, .22))) {
        error_ = idle_.error();
        return;
      }
      sleepMotionWait_ = range(4.5, 8);
    }
    if (sleepSeconds_ >= kSleepSeconds) {
      next_ = CharacterMode::Idle;
      sleepExiting_ = true;
      sleepTransitionSeconds_ = 0;
      if (!idle_.returnToCenter(1.2)) error_ = idle_.error();
    }
  } else {
    idle_.update(dt);  // Stationary center, with the original independent blink clock.
    advanceExpression(dt);
  }
}
}
