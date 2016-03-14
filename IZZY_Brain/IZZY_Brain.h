struct profile {
  float d;
  float accel;
  float decel;
  float vmax;
  float setVel;
  float deltaPos;
};

struct motor {
  int overCurrent;
  int fault;
  int vel;
};

struct comms {
  bool cmdReceived;
  bool transmissionLost;
  bool Estop;
  int type;
  int command;
  int motor;
  int parameter;
  int data1;
  int data2;
  long lastTime;
};

struct goBoolean {
  bool cueUpA;
  bool cueUpB;
  bool goA;
  bool goB;
};

struct cue {
  float standbyT;
  float standbyAT;
  float standbyDT;
  float t;
  float at;
  float dt;
  int dir;
  bool cueUp;
  bool go;
  long start;
  long elapsed;
  bool started;
};

struct time {
  long start;
  long elapsed;
  bool started;
};

typedef struct profile PROFILE;
typedef struct comms COMMS;
typedef struct goBoolean GOBOOLEAN;
typedef struct motor MOTOR;
typedef struct time TIME;
typedef struct cue CUE;
