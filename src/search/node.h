#ifndef NODE_H
#define NODE_H

#include <vector>

class Node {
private:
      int h;
      int g;
      int f;
public:
      Node();
      Node(int h, int g, int f);
      int getH();
      int getG();
      int getF();
};
#endif
