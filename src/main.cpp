
#include <fstream>
#include <GL/freeglut.h>
#include <SFML/System.hpp>
//#include <ctime>
#include <iostream>
#include <cmath>
#include "DynamicsWorld.h"
#include "OctreeTriangleMeshRigidBody.h"
#include "SphereRigidBody.h"
#include "mesh.h"
using namespace std;
DynamicsWorld world;


Mesh mesh;
OctreeTriangleMeshRigidBody octree;

sf::Clock timer;
bool rightmousedown;

void Display();
void Reshape(int w, int h);
void Mouse(int, int,int,int);
void Motion(int, int);
float camrot = 0.0f;

float randomf()
{
	return rand()/(float)RAND_MAX;
}

struct Ball {
	struct Color {
		float r, g, b;
	} color;
	SphereRigidBody body;

	Ball(float radius, float invmass, bool affectedByGravity)
		: body(radius, invmass, affectedByGravity)
	{
	}
};
std::vector<Ball*> balls;


void addBall()
{
	float radius = 3.0f;
	Ball* b = new Ball(radius, 1.0f/10.0f, true);

	b->color.r = randomf();
	b->color.g = randomf();
	b->color.b = randomf();

	static const float size = 13.0f;
	float s = size-(radius*2);

	b->body.position.x = 4;//(randomf() * s) - (s*0.5f);
	b->body.position.y = 20;//size-radius;
	b->body.position.z = 4;//(randomf() * s) - s;

	world.AddRigidBody(&b->body);

	balls.push_back(b);
}

int main(int argc, char** argv)
{
	// initiate glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(800, 600);
	glutCreateWindow("Physics engine");
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutMotionFunc(Motion);
	glutMouseFunc(Mouse);

	glClearColor(0.0, 0.0, 1.0, 0.0);
	glShadeModel(GL_SMOOTH);

	//glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);

	// add mesh
	if(!mesh.Load("terrain.txt"))
		return 0;

	// extract triangles from mesh
	std::vector<Triangle> triangles;
	for(uint i = 0; i < mesh.indices.size(); i += 3)
	{
		Triangle t;
		t.a = mesh.vertices[mesh.indices[i+0]];
		t.b = mesh.vertices[mesh.indices[i+1]];
		t.c = mesh.vertices[mesh.indices[i+2]];
		triangles.push_back(t);
	}

	// create the mesh rigidBody
	octree.Init(triangles, 10, 4);
	world.AddRigidBody(&octree);

	// set gravity
	world.SetGravity(Vector3(0, -9.8, 0));
	world.SetSimulationTimeStep(0.02);

	// set up view and projection matrics
	glMatrixMode(GL_PROJECTION);
	gluPerspective(90.0f, 1.0f, 0.01, 1000.0);

	GLfloat mat_ambient[] = { 0.3, 0.3, 0.3, 0.3 };
	GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat mat_shininess[] = { 50.0 };
	GLfloat light_position[] = { 30.0, 30.0, 30.0, 0.0 };

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	glShadeModel (GL_SMOOTH);

	glLightfv(GL_LIGHT0, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_BACK, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	timer.restart();
	glutMainLoop();

	return 0;
}

const float DEG_TO_RAD = (M_PI/180.0f);

void Display()
{
	world.Update(timer.getElapsedTime().asSeconds());
	timer.restart();

	// draw the sphere
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	Vector3 lookat;
	if(balls.size()>0)
	{
		lookat = balls.back()->body.position;
	}
	else
	{
		lookat = Vector3(0,0,0);
	}
	Vector3 campos =  Vector3(30,30,30);
	gluLookAt(campos.x, campos.y, campos.z, lookat.x, lookat.y, lookat.z, 0, 1, 0);

	//sin(camrot*DEG_TO_RAD)*30,20,cos(camrot*DEG_TO_RAD)*30
	//glCullFace(GL_FRONT);
	//glutSolidCube(size*2);
	//glCullFace(GL_BACK);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor3f(0.0, 1.0, 0.0);
	mesh.Draw();
	//mesh.DrawNormals();
	glColor3f(1.0, 1.0, 1.0);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//float t[] = {1.0,0.0,0.0,0.0};
	//glMaterialfv(GL_FRONT, GL_DIFFUSE, t);

	for(int i = 0; i < balls.size(); ++i)
	{
		Ball& b = *balls[i];
		glColor3f(b.color.r, b.color.g, b.color.b);
		glPushMatrix();
		glTranslatef(b.body.position.x, b.body.position.y, b.body.position.z);
		glutSolidSphere(b.body.radius, 32, 32);
		glPopMatrix();
	}

	glutPostRedisplay();
	glutSwapBuffers();
}

void Reshape(int width, int height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float yFov = 75.0f;
	if(height == 0.0f)
	{
		gluPerspective(yFov, width, 0.1, 10000.0);
	}
	else
	{
		gluPerspective(yFov, width / (float)height, 0.1, 10000.0);
	}
	glMatrixMode(GL_MODELVIEW);
}

void Mouse(int button,int state,int,int)
{
	if(state == GLUT_DOWN)
	{
		if(button == GLUT_LEFT)
			addBall();
		else
		{
			//for(int i = 1; i < 5; ++i)
			//	world.RemoveRigidBody(&walls[i]);
			rightmousedown = true;
		}
	}
}

int prevX = 0;
void Motion(int x, int y)
{
	camrot += (x - prevX) *0.5f;
	prevX = x;
}
