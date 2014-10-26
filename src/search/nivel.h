#ifndef NIVEL_H
#define NIVEL_H

class Nivel {
private:
      int f;
      int nodes;
      float time;

public:
      int getF();
      void setF(int f);
      int getNodes();
      void setNodes(int nodes);
      float getTime();
      void setTime(float t);
      Nivel(int f, int nodes, float time);
};

#endif
