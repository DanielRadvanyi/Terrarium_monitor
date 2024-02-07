#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <touchgfx/Color.hpp>

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
	if(temp < 23.0)
	{
		TempIndicatorPainter.setColor(touchgfx::Color::getColorFromRGB(225, 0, 0)); // Red
	}
	else
	{
		TempIndicatorPainter.setColor(touchgfx::Color::getColorFromRGB(40, 130, 40)); // Dark Green
	}

	TempValue.resizeToCurrentText();
	TempValue.invalidate(); // redraw
}

void MainScreenView::updateHum(unsigned int hum)
{
	Unicode::snprintf(HumValueBuffer, HUMVALUE_SIZE, "%d", hum);
	if(hum < 35 || hum > 85)
	{
		HumIndicatorPainter.setColor(touchgfx::Color::getColorFromRGB(225, 0, 0)); // Red
	}
	else
	{
		HumIndicatorPainter.setColor(touchgfx::Color::getColorFromRGB(40, 130, 40)); // Dark Green
	}
	HumValue.resizeToCurrentText();
	HumValue.invalidate(); // redraw
}

void MainScreenView::updateUV(float uv)
{
	Unicode::snprintf(UVValueBuffer, UVVALUE_SIZE, "%.1f", uv);
	UVValue.resizeToCurrentText();
	UVValue.invalidate(); // redraw
}
