#pragma once
#include <string>
#include <vector>
#include "Texture.h"
#include "AABB.h"

using std::string;
using std::vector;

class Mesh
{
public:
	Mesh();
	~Mesh();

	void unload();

	class VertexArray* getVertexArray() { return vertexArray; }
	const string& getShaderName() const { return shaderName; }
	float getRadius() const { return radius; }
	float getSpecularPower() const { return specularPower; }
	const AABB& getBox() const { return box; }

	void addTexture(Texture* texture);
	Texture* getTexture(int index);

	void setVertexArray(VertexArray* vertexArrayP);
	void setShaderName(const string& shaderNameP);
	void setRadius(float radiusP);
	void setSpecularPower(float specularPowerP);
	void setBox(const AABB& boxP);

private:
	vector<Texture*> textures;
	class VertexArray* vertexArray;
	string shaderName;
	float radius; // Bounding sphere radius
	float specularPower;
	AABB box;
};

