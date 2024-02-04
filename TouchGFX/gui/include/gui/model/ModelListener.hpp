#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}

    void bind(Model* m)
    {
        model = m;
    }

    virtual void setNewTemp(float temp) {}

    virtual void setNewHum(unsigned int hum) {}

    virtual void setNewUV(float uv) {}
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
