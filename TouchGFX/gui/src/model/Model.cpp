#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern "C"
{
	xQueueHandle tempMessage;
	xQueueHandle humMessage;
	xQueueHandle uvMessage;

	float tempVal;
	unsigned int humVal;
	float uvVal;
}

Model::Model() : modelListener(0)
{
	tempMessage = xQueueGenericCreate(1, sizeof(float), 0);
	humMessage = xQueueGenericCreate(1, sizeof(unsigned int), 0);
	uvMessage = xQueueGenericCreate(1, sizeof(float), 0);
}

void Model::tick()
{
	if(xQueueReceive(tempMessage, &tempVal, 0) == pdTRUE)
	{
		modelListener->setNewTemp(tempVal);
	}
	if(xQueueReceive(humMessage, &humVal, 0) == pdTRUE)
	{
		modelListener->setNewHum(humVal);
	}
	if(xQueueReceive(uvMessage, &uvVal, 0) == pdTRUE)
	{
		modelListener->setNewUV(uvVal);
	}
}
