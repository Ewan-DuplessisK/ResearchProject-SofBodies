#pragma once
#include <memory.h>
#include "Vector3.h"
#include "Quaternion.h"

class Matrix4
{
public:
	float mat[4][4];

	Matrix4()
	{
		*this = Matrix4::identity;
	}

	explicit Matrix4(float inMat[4][4])
	{
		memcpy(mat, inMat, 16 * sizeof(float));
	}

	// Cast to a const float pointer
	const float* getAsFloatPtr() const
	{
		return reinterpret_cast<const float*>(&mat[0][0]);
	}

	// Matrix multiplication (a * b)
	friend Matrix4 operator*(const Matrix4& a, const Matrix4& b)
	{
		Matrix4 retVal;
		// row 0
		retVal.mat[0][0] =
			a.mat[0][0] * b.mat[0][0] +
			a.mat[0][1] * b.mat[1][0] +
			a.mat[0][2] * b.mat[2][0] +
			a.mat[0][3] * b.mat[3][0];

		retVal.mat[0][1] =
			a.mat[0][0] * b.mat[0][1] +
			a.mat[0][1] * b.mat[1][1] +
			a.mat[0][2] * b.mat[2][1] +
			a.mat[0][3] * b.mat[3][1];

		retVal.mat[0][2] =
			a.mat[0][0] * b.mat[0][2] +
			a.mat[0][1] * b.mat[1][2] +
			a.mat[0][2] * b.mat[2][2] +
			a.mat[0][3] * b.mat[3][2];

		retVal.mat[0][3] =
			a.mat[0][0] * b.mat[0][3] +
			a.mat[0][1] * b.mat[1][3] +
			a.mat[0][2] * b.mat[2][3] +
			a.mat[0][3] * b.mat[3][3];

		// row 1
		retVal.mat[1][0] =
			a.mat[1][0] * b.mat[0][0] +
			a.mat[1][1] * b.mat[1][0] +
			a.mat[1][2] * b.mat[2][0] +
			a.mat[1][3] * b.mat[3][0];

		retVal.mat[1][1] =
			a.mat[1][0] * b.mat[0][1] +
			a.mat[1][1] * b.mat[1][1] +
			a.mat[1][2] * b.mat[2][1] +
			a.mat[1][3] * b.mat[3][1];

		retVal.mat[1][2] =
			a.mat[1][0] * b.mat[0][2] +
			a.mat[1][1] * b.mat[1][2] +
			a.mat[1][2] * b.mat[2][2] +
			a.mat[1][3] * b.mat[3][2];

		retVal.mat[1][3] =
			a.mat[1][0] * b.mat[0][3] +
			a.mat[1][1] * b.mat[1][3] +
			a.mat[1][2] * b.mat[2][3] +
			a.mat[1][3] * b.mat[3][3];

		// row 2
		retVal.mat[2][0] =
			a.mat[2][0] * b.mat[0][0] +
			a.mat[2][1] * b.mat[1][0] +
			a.mat[2][2] * b.mat[2][0] +
			a.mat[2][3] * b.mat[3][0];

		retVal.mat[2][1] =
			a.mat[2][0] * b.mat[0][1] +
			a.mat[2][1] * b.mat[1][1] +
			a.mat[2][2] * b.mat[2][1] +
			a.mat[2][3] * b.mat[3][1];

		retVal.mat[2][2] =
			a.mat[2][0] * b.mat[0][2] +
			a.mat[2][1] * b.mat[1][2] +
			a.mat[2][2] * b.mat[2][2] +
			a.mat[2][3] * b.mat[3][2];

		retVal.mat[2][3] =
			a.mat[2][0] * b.mat[0][3] +
			a.mat[2][1] * b.mat[1][3] +
			a.mat[2][2] * b.mat[2][3] +
			a.mat[2][3] * b.mat[3][3];

		// row 3
		retVal.mat[3][0] =
			a.mat[3][0] * b.mat[0][0] +
			a.mat[3][1] * b.mat[1][0] +
			a.mat[3][2] * b.mat[2][0] +
			a.mat[3][3] * b.mat[3][0];

		retVal.mat[3][1] =
			a.mat[3][0] * b.mat[0][1] +
			a.mat[3][1] * b.mat[1][1] +
			a.mat[3][2] * b.mat[2][1] +
			a.mat[3][3] * b.mat[3][1];

		retVal.mat[3][2] =
			a.mat[3][0] * b.mat[0][2] +
			a.mat[3][1] * b.mat[1][2] +
			a.mat[3][2] * b.mat[2][2] +
			a.mat[3][3] * b.mat[3][2];

		retVal.mat[3][3] =
			a.mat[3][0] * b.mat[0][3] +
			a.mat[3][1] * b.mat[1][3] +
			a.mat[3][2] * b.mat[2][3] +
			a.mat[3][3] * b.mat[3][3];

		return retVal;
	}

	Matrix4& operator*=(const Matrix4& right)
	{
		*this = *this * right;
		return *this;
	}

	// Invert the matrix - super slow
	void invert();

	Vector3 getTranslation() const
	{
		return Vector3(mat[3][0], mat[3][1], mat[3][2]);
	}

	Vector3 getXAxis() const
	{
		return Vector3::normalize(Vector3(mat[0][0], mat[0][1], mat[0][2]));
	}

	Vector3 getYAxis() const
	{
		return Vector3::normalize(Vector3(mat[1][0], mat[1][1], mat[1][2]));
	}

	Vector3 getZAxis() const
	{
		return Vector3::normalize(Vector3(mat[2][0], mat[2][1], mat[2][2]));
	}

	Vector3 getScale() const
	{
		Vector3 retVal;
		retVal.x = Vector3(mat[0][0], mat[0][1], mat[0][2]).length();
		retVal.y = Vector3(mat[1][0], mat[1][1], mat[1][2]).length();
		retVal.z = Vector3(mat[2][0], mat[2][1], mat[2][2]).length();
		return retVal;
	}

	static Matrix4 createScale(float xScale, float yScale, float zScale)
	{
		float temp[4][4] =
		{
			{ xScale, 0.0f, 0.0f, 0.0f },
			{ 0.0f, yScale, 0.0f, 0.0f },
			{ 0.0f, 0.0f, zScale, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f }
		};
		return Matrix4(temp);
	}

	static Matrix4 createScale(const Vector3& scaleVector)
	{
		return createScale(scaleVector.x, scaleVector.y, scaleVector.z);
	}

	static Matrix4 createScale(float scale)
	{
		return createScale(scale, scale, scale);
	}

	static Matrix4 createRotationX(float theta)
	{
		float temp[4][4] =
		{
			{ 1.0f, 0.0f, 0.0f , 0.0f },
			{ 0.0f, Maths::cos(theta), Maths::sin(theta), 0.0f },
			{ 0.0f, Maths::sin(theta), Maths::cos(theta), 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f },
		};
		return Matrix4(temp);
	}

	static Matrix4 createRotationY(float theta)
	{
		float temp[4][4] =
		{
			{ Maths::cos(theta), 0.0f, -Maths::sin(theta), 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f },
			{ Maths::sin(theta), 0.0f, Maths::cos(theta), 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f },
		};
		return Matrix4(temp);
	}

	static Matrix4 createRotationZ(float theta)
	{
		float temp[4][4] =
		{
			{ Maths::cos(theta), Maths::sin(theta), 0.0f, 0.0f },
			{ -Maths::sin(theta), Maths::cos(theta), 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 0.0f, 1.0f },
		};
		return Matrix4(temp);
	}

	static Matrix4 createTranslation(const Vector3& trans)
	{
		float temp[4][4] =
		{
			{ 1.0f, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 0.0f },
			{ trans.x, trans.y, trans.z, 1.0f }
		};
		return Matrix4(temp);
	}

	static Matrix4 createSimpleViewProj(float width, float height)
	{
		float temp[4][4] =
		{
			{ 2.0f / width, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 2.0f / height, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f, 1.0f }
		};
		return Matrix4(temp);
	}

	static Matrix4 createFromQuaternion(const Quaternion& q)
	{
		float mat[4][4];

		mat[0][0] = 1.0f - 2.0f * q.y * q.y - 2.0f * q.z * q.z;
		mat[0][1] = 2.0f * q.x * q.y + 2.0f * q.w * q.z;
		mat[0][2] = 2.0f * q.x * q.z - 2.0f * q.w * q.y;
		mat[0][3] = 0.0f;

		mat[1][0] = 2.0f * q.x * q.y - 2.0f * q.w * q.z;
		mat[1][1] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z;
		mat[1][2] = 2.0f * q.y * q.z + 2.0f * q.w * q.x;
		mat[1][3] = 0.0f;

		mat[2][0] = 2.0f * q.x * q.z + 2.0f * q.w * q.y;
		mat[2][1] = 2.0f * q.y * q.z - 2.0f * q.w * q.x;
		mat[2][2] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y;
		mat[2][3] = 0.0f;

		mat[3][0] = 0.0f;
		mat[3][1] = 0.0f;
		mat[3][2] = 0.0f;
		mat[3][3] = 1.0f;

		return Matrix4(mat);
	}

	static Matrix4 createLookAt(const Vector3& eye, const Vector3& target, const Vector3& up)
	{
		Vector3 zaxis = Vector3::normalize(target - eye);
		Vector3 xaxis = Vector3::normalize(Vector3::cross(up, zaxis));
		Vector3 yaxis = Vector3::normalize(Vector3::cross(zaxis, xaxis));
		Vector3 trans;
		trans.x = -Vector3::dot(xaxis, eye);
		trans.y = -Vector3::dot(yaxis, eye);
		trans.z = -Vector3::dot(zaxis, eye);

		float temp[4][4] =
		{
			{ xaxis.x, yaxis.x, zaxis.x, 0.0f },
			{ xaxis.y, yaxis.y, zaxis.y, 0.0f },
			{ xaxis.z, yaxis.z, zaxis.z, 0.0f },
			{ trans.x, trans.y, trans.z, 1.0f }
		};
		return Matrix4(temp);
	}

	static Matrix4 createOrtho(float width, float height, float near, float far)
	{
		float temp[4][4] =
		{
			{ 2.0f / width, 0.0f, 0.0f, 0.0f },
			{ 0.0f, 2.0f / height, 0.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f / (far - near), 0.0f },
			{ 0.0f, 0.0f, near / (near - far), 1.0f }
		};
		return Matrix4(temp);
	}

	static Matrix4 createPerspectiveFOV(float fovY, float width, float height, float near, float far)
	{
		float yScale = Maths::cot(fovY / 2.0f);
		float xScale = yScale * height / width;
		float temp[4][4] =
		{
			{ xScale, 0.0f, 0.0f, 0.0f },
			{ 0.0f, yScale, 0.0f, 0.0f },
			{ 0.0f, 0.0f, far / (far - near), 1.0f },
			{ 0.0f, 0.0f, -near * far / (far - near), 0.0f }
		};
		return Matrix4(temp);
	}

	static const Matrix4 identity;
};


class Matrix3
{
public:
    float mat[3][3];

    Matrix3()
    {
        *this = Matrix3::identity;
    }

    explicit Matrix3(float inMat[3][3])
    {
        memcpy(mat, inMat, 9 * sizeof(float));
    }

	// Cast to a const float pointer
	const float* getAsFloatPtr() const
    {
    	return reinterpret_cast<const float*>(&mat[0][0]);
    }

    static const Matrix3 identity;

    Vector3 getColumn(int col) const
    {
        return Vector3(mat[0][col], mat[1][col], mat[2][col]);
    }

    void setColumn(int col, const Vector3& v)
    {
        mat[0][col] = v.x;
        mat[1][col] = v.y;
        mat[2][col] = v.z;
    }

    // Matrix multiplication
    friend Matrix3 operator*(const Matrix3& a, const Matrix3& b)
    {
        Matrix3 result;
        for (int i = 0; i < 3; ++i) {       // row
            for (int j = 0; j < 3; ++j) {   // column
                result.mat[i][j] =
                    a.mat[i][0] * b.mat[0][j] +
                    a.mat[i][1] * b.mat[1][j] +
                    a.mat[i][2] * b.mat[2][j];
            }
        }
        return result;
    }

    Matrix3& operator*=(const Matrix3& right)
    {
        *this = *this * right;
        return *this;
    }

    // Orthonormalize columns (Gram-Schmidt)
    Matrix3 orthonormalized() const
    {
        Matrix3 result;

        Vector3 x = getColumn(0).normalized();
        Vector3 y = (getColumn(1) - x * Vector3::dot(getColumn(1), x)).normalized();
        Vector3 z = Vector3::cross(x, y); // ensures a right-handed basis

        result.setColumn(0, x);
        result.setColumn(1, y);
        result.setColumn(2, z);

        return result;
    }

    // Outer product of two vectors: returns a matrix
    static Matrix3 outerProduct(const Vector3& a, const Vector3& b)
    {
        Matrix3 result;
        result.mat[0][0] = a.x * b.x;
        result.mat[0][1] = a.x * b.y;
        result.mat[0][2] = a.x * b.z;

        result.mat[1][0] = a.y * b.x;
        result.mat[1][1] = a.y * b.y;
        result.mat[1][2] = a.y * b.z;

        result.mat[2][0] = a.z * b.x;
        result.mat[2][1] = a.z * b.y;
        result.mat[2][2] = a.z * b.z;

        return result;
    }

    Matrix3& operator+=(const Matrix3& other)
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                mat[r][c] += other.mat[r][c];
        return *this;
    }

    void zero()
    {
        memset(mat, 0, 9 * sizeof(float));
    }

	Matrix3 polarDecomposition() const {
    	// Placeholder: just orthonormalize columns using Gram-Schmidt
    	Matrix3 R = *this;
    	R=R.orthonormalized();
    	return R;
    }
};
