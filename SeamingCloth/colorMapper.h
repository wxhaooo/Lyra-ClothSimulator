#pragma once

#include"Common.h"
#include"colormaps.h"

#include <cmath>

#include<glm\gtc\matrix_transform.hpp>

namespace Lyra
{
	enum ColorMap;
	template<typename T> class ColorMapper;
}

namespace Lyra
{
	enum ColorMap
	{
		COLOR_NONE = 0,
		COLOR_GRAYSCALE,
		COLOR_RAINBOW,
		COLOR_AUTUMN, // varies smoothly from red, through orange, to yellow.
		COLOR_BONE, // is a grayscale colormap with a higher value for the blue component. This colormap is useful for adding an "electronic" look to grayscale images.
		COLOR_COLORCUBE, // contains as many regularly spaced colors in RGB colorspace as possible, while attempting to provide more steps of gray, pure red, pure green, and pure blue
		COLOR_COOL, // consists of colors that are shades of cyan and magenta. It varies smoothly from cyan to magenta.
		COLOR_COPPER, // varies smoothly from black to bright copper.
		COLOR_FLAG, // consists of the colors red, white, blue, and black. This colormap completely changes color with each index increment.
		COLOR_HOT, // varies smoothly from black, through shades of red, orange, and yellow, to white.
		COLOR_HSV, // varies the hue component of the hue-saturation-value color model. The colors begin with red, pass through yellow, green, cyan, blue, magenta, and return to red. The colormap is particularly appropriate for displaying periodic functions. hsv(m) is the same as hsv2rgb([h ones(m,2)]) where h is the linear ramp, h = (0:m-1)'/m.
		COLOR_JET, // ranges from blue to red, and passes through the colors cyan, yellow, and orange. It is a variation of the hsv colormap. The jet colormap is associated with an astrophysical fluid jet simulation from the National Center for Supercomputer Applications.
		COLOR_LINES, // produces a colormap of colors specified by the axes ColorOrder property and a shade of gray.
		COLOR_PINK, // contains pastel shades of pink. The pink colormap provides sepia tone colorization of grayscale photographs.
		COLOR_PRISM, // repeats the six colors red, orange, yellow, green, blue, and violet.
		COLOR_SPRING, // consists of colors that are shades of magenta and yellow.
		COLOR_SUMMER, // consists of colors that are shades of green and yellow.
		COLOR_WINTER, // consists of colors that are shades of blue and green.
		COLOR__MAGMA,
		COLOR__PLASMA,
		COLOR__INFERNO,
		COLOR_VIRIDIS,
		COLOR_CM,
		COLOR_NUMBER

	};
}

namespace Lyra
{
	template<typename T>
	class ColorMapper
	{
	public:
		ColorMapper();
		~ColorMapper() = default;

		void GetGlmColor(T v, ColorMap cmap, glm::vec<3, T>& color, bool inv = false);

	private:
		T color_grayscale[256 * 3];
		T color_rainbow[256 * 3];
		T *colormaps[COLOR_NUMBER];
	private:
		static void rainbow(T v, T *col);
	};
}

template<typename T>
Lyra::ColorMapper<T>::ColorMapper()
	//-----------------------------------------------------------------------------
{
	int i;

	for (i = 0; i < 256; i++)
		rainbow((T)i / 255.0f, color_rainbow + 3 * i);

	for (i = 0; i < 256; i++)
	{
		color_grayscale[3 * i] = (T)i / 255.0f;
		color_grayscale[3 * i + 1] = (T)i / 255.0f;
		color_grayscale[3 * i + 2] = (T)i / 255.0f;
	}

	for (i = 1; i < COLOR_NUMBER; i++)
	{
		switch (i)
		{
		case COLOR_RAINBOW: colormaps[i] = color_rainbow; break;
		case COLOR_GRAYSCALE: colormaps[i] = color_grayscale; break;
		case COLOR_AUTUMN: colormaps[i] = color_autumn; break;
		case COLOR_BONE: colormaps[i] = color_bone; break;
		case COLOR_COLORCUBE: colormaps[i] = color_colorcube; break;
		case COLOR_COOL: colormaps[i] = color_cool; break;
		case COLOR_COPPER: colormaps[i] = color_copper; break;
		case COLOR_FLAG: colormaps[i] = color_flag; break;
		case COLOR_HOT: colormaps[i] = color_hot; break;
		case COLOR_HSV: colormaps[i] = color_hsv; break;
		case COLOR_JET: colormaps[i] = color_jet; break;
		case COLOR_LINES: colormaps[i] = color_lines; break;
		case COLOR_PINK: colormaps[i] = color_pink; break;
		case COLOR_PRISM: colormaps[i] = color_prism; break;
		case COLOR_SPRING: colormaps[i] = color_spring; break;
		case COLOR_SUMMER: colormaps[i] = color_summer; break;
		case COLOR_WINTER: colormaps[i] = color_winter; break;
		case COLOR__MAGMA: colormaps[i] = _magma_data; break;
		case COLOR__PLASMA:colormaps[i] = _plasma_data; break;
		case COLOR__INFERNO:colormaps[i] = _inferno_data; break;
		case COLOR_VIRIDIS:colormaps[i] = _viridis_data; break;
		case COLOR_CM:colormaps[i] = cm_data; break;
		}
	}
}

template<typename T>
void Lyra::ColorMapper<T>::rainbow(T v, T* cols) {
	cols[0] = 1;
	cols[1] = 1;
	cols[2] = 1;

	if (v < 0.25f)
	{
		cols[0] = 0;
		cols[1] = 4 * v;
	}
	else if (v < 0.5f)
	{
		cols[0] = 0;
		cols[2] = 1 + 4 * (0.25f - v);
	}
	else if (v < 0.75f)
	{
		cols[0] = 4 * (v - 0.5f);
		cols[2] = 0;
	}
	else
	{
		cols[1] = 1 + 4 * (0.75f - v);
		cols[2] = 0;
	}
}

template<typename T>
void Lyra::ColorMapper<T>::GetGlmColor(T v, ColorMap cmap, glm::vec<3,T>& color, bool inv) {
	if (cmap != COLOR_NONE && cmap != COLOR_NUMBER)
	{
		int i = (int)((fabs(v)) * 255.0);//sqrt
		if (i > 255) i = 255;
		if (inv) i = 255 - i;
		color.x = colormaps[cmap][3 * i];
		color.y = colormaps[cmap][3 * i + 1];
		color.z = colormaps[cmap][3 * i + 2];
	}
}

