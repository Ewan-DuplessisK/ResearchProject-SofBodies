#include "Spline.h"

Vector3 Spline::compute(size_t index, float t) const
{
    if (index >= controlPoints.size())
    {
        return controlPoints.back();
    }
    else if (index == 0)
    {
        return controlPoints[index];
    }
    else if (index + 2 >= controlPoints.size())
    {
        return controlPoints[index];
    }

    Vector3 p0 = controlPoints[index - 1];
    Vector3 p1 = controlPoints[index];
    Vector3 p2 = controlPoints[index + 1];
    Vector3 p3 = controlPoints[index + 2];
    // Compute position according to Catmull-Rom equation
    Vector3 position = 0.5f * ((2.0f * p1) + (-1.0f * p0 + p2) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t * t +
        (-1.0f * p0 + 3.0f * p1 - 3.0f * p2 + p3) * t * t * t);
    return position;
}
