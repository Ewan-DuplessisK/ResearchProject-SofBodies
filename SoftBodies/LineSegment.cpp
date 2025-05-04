#include "LineSegment.h"

LineSegment::LineSegment(const Vector3& startP, const Vector3& endP) : start(startP), end(endP)
{
}

Vector3 LineSegment::pointOnSegment(float t) const
{
	return start + (end - start) * t;
}

float LineSegment::minDistSq(const Vector3& point) const
{
	// Construct vectors
	Vector3 ab = end - start;
	Vector3 ba = -1.0f * ab;
	Vector3 ac = point - start;
	Vector3 bc = point - end;

	// Case 1: C projects prior to A
	if (Vector3::dot(ab, ac) < 0.0f)
	{
		return ac.lengthSq();
	}
	// Case 2: C projects after B
	else if (Vector3::dot(ba, bc) < 0.0f)
	{
		return bc.lengthSq();
	}
	// Case 3: C projects onto line
	else
	{
		// Compute p
		float scalar = Vector3::dot(ac, ab) / Vector3::dot(ab, ab);
		Vector3 p = scalar * ab;
		// Compute length squared of ac - p
		return (ac - p).lengthSq();
	}
}

float LineSegment::minDistSq(const LineSegment& s1, const LineSegment& s2)
{
	Vector3 u = s1.end - s1.start;
	Vector3 v = s2.end - s2.start;
	Vector3 w = s1.start - s2.start;
	float a = Vector3::dot(u, u);         // always >= 0
	float b = Vector3::dot(u, v);
	float c = Vector3::dot(v, v);         // always >= 0
	float d = Vector3::dot(u, w);
	float e = Vector3::dot(v, w);
	float D = a * c - b * b;        // always >= 0
	float sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	float tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

	// compute the line parameters of the two closest points
	if (Maths::nearZero(D)) 
	{
		// the lines are almost parallel
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	// get the closest points on the infinite lines
	else 
	{                 
		sN = (b * e - c * d);
		tN = (a * e - b * d);
		if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	// tc < 0 => the t=0 edge is visible
	if (tN < 0.0) {            
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	// tc > 1  => the t=1 edge is visible
	else if (tN > tD) 
	{      
		tN = tD;
		// recompute sc for this edge
		if (-d + b < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else 
		{
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (Maths::nearZero(sN) ? 0.0f : sN / sD);
	tc = (Maths::nearZero(tN) ? 0.0f : tN / tD);

	// get the difference of the two closest points
	Vector3   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

	return dP.lengthSq();   // return the closest distance squared
}
