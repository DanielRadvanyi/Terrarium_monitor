#include <gui/mainscreen_screen/MainScreenView.hpp>

MainScreenView::MainScreenView()
{

}

void MainScreenView::setupScreen()
{
    MainScreenViewBase::setupScreen();
}

void MainScreenView::tearDownScreen()
{
    MainScreenViewBase::tearDownScreen();
}

void MainScreenView::updateTemp(float temp)
{
	Unicode::snprintf(TempValueBuffer, TEMPVALUE_SIZE, "%.1f", temp);
	TempValue.resizeToCurrentText();
	TempValue.invalidate(); // redraw
}

void MainScreenView::updateHum(unsigned int hum)
{
	Unicode::snprintf(HumValueBuffer, HUMVALUE_SIZE, "%d", hum);
	HumValue.resizeToCurrentText();
	HumValue.invalidate(); // redraw
}

void MainScreenView::updateUV(float uv)
{
	Unicode::snprintf(UVValueBuffer, UVVALUE_SIZE, "%.1f", uv);
	UVValue.resizeToCurrentText();
	UVValue.invalidate(); // redraw
}
