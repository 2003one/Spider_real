#ifndef SPIDER_CALIBRATION_H
#define SPIDER_CALIBRATION_H

//======================================================
// CALIBRATION TARGET POSE (from spider_bot.urdf)
//
// Servo angle 0 rad (= 90 deg, pulse ~300) must equal
// the URDF zero configuration:
//
//   coxa  : leg perpendicular to the body side
//   femur : pointing STRAIGHT DOWN (vertical)
//   tibia : in line with femur (leg fully extended)
//
//        body
//     ────┬────
//         │  coxa (sideways)
//         ├──
//         │
//         │  femur (straight down)
//         │
//         │  tibia (straight down)
//         │
//         ▼  foot
//
// Procedure:
//  1. Set CALIBRATION_MODE 1 in Config.h, flash.
//  2. Prop the body up so legs hang freely.
//  3. Remount every horn so each leg matches the
//     pose above as closely as the spline allows.
//  4. Fine-tune the remaining error with
//     SERVO_OFFSET below (degrees), re-flash,
//     repeat until all four legs are straight.
//======================================================

//--------------------------------------------------
// Servo Directions
//
//  1  = Normal
// -1  = Reversed
//
// Positive femur rotation must swing the leg
// OUTWARD+UP on left legs and the mirror on right
// legs. If a joint moves the wrong way during the
// gait, flip its sign here.
//
// NOTE: every entry must be +1 or -1. A value of 0
// multiplies that joint's angle by zero and freezes
// the servo at neutral.
//--------------------------------------------------

const int SERVO_DIRECTION[12] =
{
     1, -1, 1,      // Front Left   (ch 0,1,2)
     -1, -1, 1,      // Front Right  (ch 3,4,5)  ← femur + tibia flipped
     1, 1, -1,      // Rear Left    (ch 6,7,8)
     -1, -1, -1       // Rear Right   (ch 9,10,11) ← femur + tibia flipped
};

//--------------------------------------------------
// Servo Offsets (degrees)
//
// Correction for horn mounting error, applied
// after direction. Positive/negative rotates
// the joint either way at neutral.
//--------------------------------------------------

const float SERVO_OFFSET[12] =
{
     0, 0, 0,      // FL
     0, 0, 0,      // FR
     0, 0, 0,      // RL
     0, 0, 0       // RR
};

#endif
