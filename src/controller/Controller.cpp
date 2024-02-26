#include "Controller.h"
#include <memory>
#include <algorithm>
#include "public/bridge/consolevariablebridge.h"
#if __APPLE__
#include <SDL_events.h>
#else
#include <SDL2/SDL_events.h>
#endif
#include <spdlog/spdlog.h>

#define M_TAU 6.2831853071795864769252867665590057 // 2 * pi
#define MINIMUM_RADIUS_TO_MAP_NOTCH 0.9

namespace LUS {

Controller::Controller(int32_t deviceIndex) : mIsRumbling(false), mLedColor({ 0, 0, 0 }), mDeviceIndex(deviceIndex) {
    for (int32_t portIndex = 0; portIndex < MAXCONTROLLERS; portIndex++) {
        mProfiles[portIndex] = std::make_shared<DeviceProfile>();
        mButtonData[portIndex] = std::make_shared<Buttons>();
    }
}

Controller::~Controller() {
    SPDLOG_TRACE("destruct controller");
}

double Controller::ReadStick(int32_t portIndex, Stick stick, Axis axis) {
    switch (stick) {
        case Stick::LEFT: {
            switch (axis) {
                case Axis::X: {
                    if (GetLeftStickX(portIndex) == 0) {
                        if (GetPressedButtons(portIndex) & BTN_STICKLEFT) {
                            return -1.0;
                        } else if (GetPressedButtons(portIndex) & BTN_STICKRIGHT) {
                            return 1.0;
                        }
                    } else {
                        return GetLeftStickX(portIndex);
                    }
                    break;
                }
                case Axis::Y: {
                    if (GetLeftStickY(portIndex) == 0) {
                        if (GetPressedButtons(portIndex) & BTN_STICKDOWN) {
                            return -1.0;
                        } else if (GetPressedButtons(portIndex) & BTN_STICKUP) {
                            return 1.0;
                        }
                    } else {
                        return GetLeftStickY(portIndex);
                    }
                    break;
                }
            }
            break;
        }
        case Stick::RIGHT: {
            switch (axis) {
                case Axis::X: {
                    if (GetRightStickX(portIndex) == 0) {
                        if (GetPressedButtons(portIndex) & BTN_VSTICKLEFT) {
                            return -1.0;
                        } else if (GetPressedButtons(portIndex) & BTN_VSTICKRIGHT) {
                            return 1.0;
                        }
                    } else {
                        return GetRightStickX(portIndex);
                    }
                    break;
                }
                case Axis::Y: {
                    if (GetRightStickY(portIndex) == 0) {
                        if (GetPressedButtons(portIndex) & BTN_VSTICKDOWN) {
                            return -1.0;
                        } else if (GetPressedButtons(portIndex) & BTN_VSTICKUP) {
                            return 1.0;
                        }
                    } else {
                        return GetRightStickY(portIndex);
                    }
                    break;
                }
            }
            break;
        }
    }

    return 0;
}

void Controller::ProcessStick(double& x, double& y, double deadzoneX, double deadzoneY, int32_t notchProxmityThreshold) {
    const double maxRadiusOuterDeadzone = sqrt(2.0) * (MAX_DIAGONAL / MAX_AXIS_RANGE * (MAX_AXIS_RANGE - deadzoneX) + deadzoneX);
    //printf("maxRadiusOuterDeadzone: %2.15f \n", maxRadiusOuterDeadzone); //full precision but inexact
    //printf("deadzoneX: %2.15f \n", deadzoneX); //looks okay with keyboard's 0.0
    //printf("x initial: %2.15f\t", x);
    //printf("y initial: %2.15f\n", y);
    auto ux = x * maxRadiusOuterDeadzone;
    auto uy = y * maxRadiusOuterDeadzone;
    //printf("ux initial: %2.15f\t", ux);
    //printf("uy initial: %2.15f\n", uy);

    // TODO: handle deadzones separately for X and Y
    if (deadzoneX != deadzoneY) {
        SPDLOG_TRACE("Invalid Deadzone configured. Up/Down was {} and Left/Right is {}", deadzoneY, deadzoneX);
    }

    // create outer circular dead-zone and scale linearly with respect to an inner axial deadzone
    auto len = hypot(ux, uy);
    if (len <= maxRadiusOuterDeadzone) {
        auto lenX = fabs(ux);
        auto lenY = fabs(uy);
        if (lenX <= deadzoneX) {
            lenX = 0.0;
        } else {
            lenX = (lenX - deadzoneX) * MAX_AXIS_RANGE / (MAX_AXIS_RANGE - deadzoneX) / lenX;
        }
        ux *= lenX;
        if (lenY <= deadzoneX) {
            lenY = 0.0;
        } else {
            lenY = (lenY - deadzoneX) * MAX_AXIS_RANGE / (MAX_AXIS_RANGE - deadzoneX) / lenY;
        }
        uy *= lenY;
    } else {
        len = maxRadiusOuterDeadzone / len;
        ux *= len;
        uy *= len;
    }
    //printf("ux post-scale: %2.15f\t", ux);
    //printf("uy post-scale: %2.15f\n", uy);

    // map to virtual notches
    const double notchProximityValRadians = notchProxmityThreshold * M_TAU / 360.0;

    const double distance = std::sqrt((ux * ux) + (uy * uy)) / maxRadiusOuterDeadzone;
    if (distance >= MINIMUM_RADIUS_TO_MAP_NOTCH) {
        auto angle = atan2(uy, ux) + M_TAU;
        auto newAngle = GetClosestNotch(angle, notchProximityValRadians);

        ux = cos(newAngle) * distance * maxRadiusOuterDeadzone;
        uy = sin(newAngle) * distance * maxRadiusOuterDeadzone;
    }
    //printf("ux post-notch: %2.15f\t", ux);
    //printf("uy post-notch: %2.15f\n", uy);

    // bound diagonals to an octagonal range {-MAX_DIAGONAL ... +MAX_DIAGONAL}
    if (ux != 0.0 && uy != 0.0) {
        auto slope = uy / ux;
        auto edgex = copysign(MAX_AXIS_RANGE / (fabs(slope) + (MAX_AXIS_RANGE - MAX_DIAGONAL) / MAX_DIAGONAL), ux);
        auto edgey = copysign(std::min(fabs(edgex * slope), MAX_AXIS_RANGE / (1.0 / fabs(slope) + (MAX_AXIS_RANGE - MAX_DIAGONAL) / MAX_DIAGONAL)), uy);
        edgex = edgey / slope;

        auto lengthCurrent = hypot (ux, uy);
        auto distanceEdge = hypot (edgex, edgey);
        if (lengthCurrent > distanceEdge) {
            ux = edgex;
            uy = edgey;
        }
    }
    //printf("ux post-octagon: %2.15f\t", ux);
    //printf("uy post-octagon: %2.15f\n", uy);

    // do not exceed MAX_AXIS_RANGE
    if (fabs(ux) > MAX_AXIS_RANGE) ux = copysign(MAX_AXIS_RANGE, ux);
    if (fabs(uy) > MAX_AXIS_RANGE) uy = copysign(MAX_AXIS_RANGE, uy);
    //printf("ux post-clamp: %2.15f\t", ux);
    //printf("uy post-clamp: %2.15f\n", uy);

    // counteract possible inexact result produced by maxRadiusOuterDeadzone with an epsilon
    ux = copysign(fabs(ux) + 1e-09, ux);
    uy = copysign(fabs(uy) + 1e-09, uy);
    //printf("ux post-epsilon: %2.15f\t", ux);
    //printf("uy post-epsilon: %2.15f\n", uy);

    // assign back to original sign
    x = ux;
    y = uy;
    //printf ("x final: %2.15f\t", x);
    //printf ("y final: %2.15f\n", y);
}

void Controller::ReadToPad(OSContPad* pad, int32_t portIndex) {
    ReadDevice(portIndex);

    OSContPad padToBuffer = { 0 };

#ifndef __WIIU__
    SDL_PumpEvents();
#endif

    // Button Inputs
    padToBuffer.button |= GetPressedButtons(portIndex) & 0xFFFF;

    // Stick Inputs
    double leftStickX = ReadStick(portIndex, LEFT, X);
    double leftStickY = ReadStick(portIndex, LEFT, Y);
    double rightStickX = ReadStick(portIndex, RIGHT, X);
    double rightStickY = ReadStick(portIndex, RIGHT, Y);

    auto profile = GetProfile(portIndex);
    ProcessStick(leftStickX, leftStickY, profile->AxisDeadzones[0], profile->AxisDeadzones[1],
                 profile->NotchProximityThreshold);
    ProcessStick(rightStickX, rightStickY, profile->AxisDeadzones[2], profile->AxisDeadzones[3],
                 profile->NotchProximityThreshold);

    if (pad == nullptr) {
        return;
    }

    padToBuffer.stick_x = leftStickX;
    padToBuffer.stick_y = leftStickY;
    padToBuffer.right_stick_x = rightStickX;
    padToBuffer.right_stick_y = rightStickY;
    //printf("stick_x: %d\t", padToBuffer.stick_x);
    //printf("stick_y: %d\n\n", padToBuffer.stick_y);

    // Gyro
    padToBuffer.gyro_x = GetGyroX(portIndex);
    padToBuffer.gyro_y = GetGyroY(portIndex);

    mPadBuffer.push_front(padToBuffer);
    if (pad != nullptr) {
        auto& padFromBuffer =
            mPadBuffer[std::min(mPadBuffer.size() - 1, (size_t)CVarGetInteger("gSimulatedInputLag", 0))];
        pad->button |= padFromBuffer.button;
        if (pad->stick_x == 0) {
            pad->stick_x = padFromBuffer.stick_x;
        }
        if (pad->stick_y == 0) {
            pad->stick_y = padFromBuffer.stick_y;
        }
        if (pad->gyro_x == 0) {
            pad->gyro_x = padFromBuffer.gyro_x;
        }
        if (pad->gyro_y == 0) {
            pad->gyro_y = padFromBuffer.gyro_y;
        }
        if (pad->right_stick_x == 0) {
            pad->right_stick_x = padFromBuffer.right_stick_x;
        }
        if (pad->right_stick_y == 0) {
            pad->right_stick_y = padFromBuffer.right_stick_y;
        }
    }

    while (mPadBuffer.size() > 6) {
        mPadBuffer.pop_back();
    }
}

void Controller::SetButtonMapping(int32_t portIndex, int32_t deviceButtonId, int32_t n64bitmask) {
    GetProfile(portIndex)->Mappings[deviceButtonId] = n64bitmask;
}

double& Controller::GetLeftStickX(int32_t portIndex) {
    return mButtonData[portIndex]->LeftStickX;
}

double& Controller::GetLeftStickY(int32_t portIndex) {
    return mButtonData[portIndex]->LeftStickY;
}

double& Controller::GetRightStickX(int32_t portIndex) {
    return mButtonData[portIndex]->RightStickX;
}

double& Controller::GetRightStickY(int32_t portIndex) {
    return mButtonData[portIndex]->RightStickY;
}

int32_t& Controller::GetPressedButtons(int32_t portIndex) {
    return mButtonData[portIndex]->PressedButtons;
}

float& Controller::GetGyroX(int32_t portIndex) {
    return mButtonData[portIndex]->GyroX;
}

float& Controller::GetGyroY(int32_t portIndex) {
    return mButtonData[portIndex]->GyroY;
}

std::shared_ptr<DeviceProfile> Controller::GetProfile(int32_t portIndex) {
    return mProfiles[portIndex];
}

bool Controller::IsRumbling() {
    return mIsRumbling;
}

Color_RGB8 Controller::GetLedColor() {
    return mLedColor;
}

std::string Controller::GetGuid() {
    return mGuid;
}

std::string Controller::GetControllerName() {
    return mControllerName;
}

double Controller::GetClosestNotch(double angle, double approximationThreshold) {
    constexpr auto octagonAngle = M_TAU / 8.0;
    const auto closestNotch = std::round(angle / octagonAngle) * octagonAngle;
    const auto distanceToNotch = std::abs(fmod(closestNotch - angle + M_PI, M_TAU) - M_PI);
    return distanceToNotch < approximationThreshold / 2.0 ? closestNotch : angle;
}
} // namespace LUS
