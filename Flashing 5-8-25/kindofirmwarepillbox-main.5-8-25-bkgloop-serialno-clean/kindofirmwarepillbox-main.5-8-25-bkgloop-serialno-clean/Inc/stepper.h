



#ifndef _STEPPER_H
#define _STEPPER_H

#include "main.h"

#define REFVOLMSB

typedef struct {
  uint32_t steps;
  uint16_t am;
  uint16_t vm;
  uint16_t freq;
  uint32_t cnt;
  uint32_t timeInit;
  float time;
  float h;
  float a;
  float v;
  float s;
  float vhT;
  float amT;
  float t[8];
  float at[8];
  float vt[8];
  float st[8];
} stepper_t;
extern stepper_t *stp;
extern stepper_t stepper[5];

void startstepper(uint8_t i, uint8_t dir);

void setAlldac(void);
void setdac(uint8_t i);
void setdacstop(uint8_t i);

void stepperdir(uint8_t dir);

void assignstepper(uint8_t i,uint8_t v);
void StopStp(uint8_t i);

void stepInit(uint8_t i, uint32_t step);
void stepCtrl(uint8_t i);
void simplestep(uint8_t i, uint32_t step);

static inline float a1 (float u) { return stp->h*u; }
static inline float a2 (float u) { return stp->h*stp->t[1]; }
static inline float a3 (float u) { return stp->h*stp->t[1] - stp->h*u; }
static inline float a4 (float u) { return 0; }
static inline float a5 (float u) { return -stp->h*u; }
static inline float a6 (float u) { return -stp->am; }
static inline float a7 (float u) { return -stp->am + stp->h*u; }

static inline float v1 (float u) { return stp->h*u*u/2;  }
static inline float v2 (float u) { return stp->vt[1] + stp->a*u;  }
static inline float v3 (float u) { return stp->vt[2] + stp->h*stp->t[1]*u - stp->h*u*u/2;  }
static inline float v4 (float u) { return stp->vt[3];  }
static inline float v5 (float u) { return stp->vt[4] - stp->h*u*u/2;  }
static inline float v6 (float u) { return stp->vt[5] + stp->a*u;  }
static inline float v7 (float u) { return stp->vt[6] - stp->h*stp->t[1]*u + stp->h*u*u/2;  }
// cbrt( 4.5*stp->h*stp->s*stp->s );

static inline float s1 (float u) { return stp->h*u*u*u/6;  }
static inline float s2 (float u) { return stp->vt[1]*u + stp->h*stp->t[1]*u*u/2;  }
static inline float s3 (float u) { return stp->vt[2]*u + stp->h*stp->t[1]*u*u/2 - stp->h*u*u*u/6; }
static inline float s4 (float u) { return stp->vt[3]*u;  }
static inline float s5 (float u) { return stp->vt[4]*u - stp->h*u*u*u/6;  }
static inline float s6 (float u) { return stp->vt[5]*u - stp->h*stp->t[1]*u*u/2;  }
static inline float s7 (float u) { return stp->vt[6]*u - stp->h*stp->t[1]*u*u/2 + stp->h*u*u*u/6;  }


#endif
