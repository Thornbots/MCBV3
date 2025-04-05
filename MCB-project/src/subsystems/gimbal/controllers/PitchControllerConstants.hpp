constexpr float KB = 0.716;  // V-rad/s
constexpr float RA = 8.705;  // ohm

#if defined(HERO)

constexpr float RATIO = 1;         // unitless
constexpr float VOLT_MAX = 22.2;   // V
constexpr float ACCEL_MAX = 40.0;  // rad/s

// Position controller constants
constexpr float KP = 15;  // sec^-1

// Feedforward constants
constexpr float KSTATIC = 0.1;  // A
constexpr float KF = 0.05;      //-0.001;                     // A

// Velocity feedback
constexpr float KPV = 0.5;                      // 0.3                  // A-s/rad
constexpr float KIV = 6;                        // 2                 // A/rad
constexpr float IV_MAX = 0.1;                   // units TBD
constexpr float INT_THRESH = VOLT_MAX * 0.85l;  // V
constexpr float TAKEBACK = 0.01;                // unitless
#elif defined(SENTRY)

constexpr float RATIO = 2;         // unitless
constexpr float VOLT_MAX = 24;   // V
constexpr float ACCEL_MAX = 240.0;  // rad/s^2

// Position controller constants
constexpr float KP = 14;  // sec^-1

// Feedforward constants
constexpr float KSTATIC = 0.1;  // A
constexpr float KF = 0.05;      //-0.001;                     // A

// Velocity feedback
constexpr float KPV = 0.2;                      // 0.3                  // A-s/rad
constexpr float KIV = 6;                        // 2                 // A/rad
constexpr float IV_MAX = 1.5 / KIV;             // A
constexpr float INT_THRESH = VOLT_MAX * 0.85l;  // V
constexpr float TAKEBACK = 0.01;                // unitless
#elif defined(INFANTRY)

constexpr float RATIO = 1;         // unitless
constexpr float VOLT_MAX = 24;   // V
constexpr float ACCEL_MAX = 240.0;  // rad/s^2

// Position controller constants
constexpr float KP = 12;  // sec^-1

// Feedforward constants
constexpr float KSTATIC = 0.1;  // A
constexpr float KF = 0.05;      //-0.001;                     // A

// Velocity feedback
constexpr float KPV = 0.4;                      // 0.3                  // A-s/rad
constexpr float KIV = 6;                        // 2                 // A/rad
constexpr float IV_MAX = 1.5 / KIV;             // A
constexpr float INT_THRESH = VOLT_MAX * 0.85l;  // V
constexpr float TAKEBACK = 0.01;                // unitless
#else

constexpr float RATIO = 1;         // unitless
constexpr float VOLT_MAX = 22.2;   // V
constexpr float ACCEL_MAX = 40.0;  // rad/s

// Position controller constants
constexpr float KP = 15;  // sec^-1

// Feedforward constants
constexpr float KSTATIC = 0.1;  // A
constexpr float KF = 0.05;      //-0.001;                     // A

// Velocity feedback
constexpr float KPV = 0.5;                      // 0.3                  // A-s/rad
constexpr float KIV = 6;                        // 2                 // A/rad
constexpr float IV_MAX = 0.1;                   // units TBD
constexpr float INT_THRESH = VOLT_MAX * 0.85l;  // V
constexpr float TAKEBACK = 0.01;                // unitless
#endif

// calculated from existing constants or robot independent (put after ifdef)
constexpr float VELO_MAX = VOLT_MAX / (KB * RATIO);  // rad/s
constexpr float KV = KB * RATIO;                     // V-s/rad
