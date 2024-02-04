#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <gui/mainscreen_screen/MainScreenPresenter.hpp>

MainScreenPresenter::MainScreenPresenter(MainScreenView& v)
    : view(v)
{

}

void MainScreenPresenter::activate()
{

}

void MainScreenPresenter::deactivate()
{

}

void MainScreenPresenter::setNewTemp(float temp)
{
	view.updateTemp(temp);
}

void MainScreenPresenter::setNewHum(unsigned int hum)
{
	view.updateHum(hum);
}

void MainScreenPresenter::setNewUV(float uv)
{
	view.updateUV(uv);
}
