#include <iostream>
#include <vector>

using namespace std;

class Base {
   public:
    virtual void logout() { cout << "This is Base" << endl; }
};

class Parent : public Base {
   public:
    virtual void logout() { cout << "This is Parernt" << endl; }

    virtual void Dfun() {
        cout << "This is Parernt function not in Base" << endl;
    }
};

class GrandParent : public Parent {
   public:
    virtual void logout() { cout << "This is GrandParent" << endl; }

    virtual void Dfun() {
        cout << "This is GrandParent derived from Parent" << endl;
    }
};

void Pacout(Base* pb) {
    pb->logout();
    Parent* pp= dynamic_cast<Parent*>(pb);
    // Parent* pp= static_cast<Parent*>(pb);
    if (pp){
        pp->Dfun();
    }
}

int main() {
    Base base;
    Parent parent;
    GrandParent grandparent;

    vector<Base*> pointTobases;
    pointTobases.resize(3);
    pointTobases[0] = &base;
    pointTobases[1] = &parent;
    pointTobases[2] = &grandparent;

    for (auto p : pointTobases) {
        Pacout(p);
    }

    return 0;
}
