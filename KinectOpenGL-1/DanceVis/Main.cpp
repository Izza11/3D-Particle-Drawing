#include <windows.h>
#include <iostream>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glext.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "InitShader.h"
#include "LoadTexture.h"
#include "imgui_impl_glut.h"
#include <vector>
#include <iostream>
#include "Cube.h"

#if defined WIN32 || _WIN32
#include <Kinect.h>
#endif

#include "glm/gtc/type_ptr.hpp"
#include "GL/freeglut.h"
#include "glm/glm.hpp"

static const int kinectDepthHeight = 424;
static const int kinectDepthWidth = 512;

static IMultiSourceFrameReader * reader = NULL;
static ICoordinateMapper * mapper = NULL;
static IKinectSensor * sensor = NULL;

static BOOLEAN bodyTracked = false;
static Joint joints[JointType_Count];

template <typename T>
inline void allocateData(T ** data, int size) { *data = new T[size]; }

template <typename T>
inline void destroyData(T ** data) { if (*data != NULL) delete[] data; }

int nFrames = 1; //how many frames of skeleton data to save

std::vector<glm::vec3> skelBuffer;
std::vector<glm::vec3> skelNormal;

std::vector<unsigned int> skelIndices;

GLuint SkelVertsVBO = -1;
GLuint SkelNormalsVBO = -1;
GLuint SkelIndex = -1;
glm::mat4 P;
glm::mat4 V;
float cam_height = 0.0f;
const int nBones = 20;

// why 2 values for each joint ???
const int skel[2 * nBones] = { JointType_Head, JointType_Neck, JointType_Neck, JointType_SpineShoulder, JointType_SpineShoulder, JointType_SpineMid, JointType_SpineMid, JointType_SpineBase,
JointType_SpineShoulder, JointType_ShoulderLeft, JointType_ShoulderLeft, JointType_ElbowLeft, JointType_ElbowLeft, JointType_WristLeft, JointType_WristLeft, JointType_HandLeft,
JointType_SpineBase, JointType_HipLeft, JointType_HipLeft, JointType_KneeLeft, JointType_KneeLeft, JointType_AnkleLeft, JointType_AnkleLeft, JointType_FootLeft,
JointType_SpineShoulder, JointType_ShoulderRight, JointType_ShoulderRight, JointType_ElbowRight, JointType_ElbowRight, JointType_WristRight, JointType_WristRight, JointType_HandRight,
JointType_SpineBase, JointType_HipRight, JointType_HipRight, JointType_KneeRight, JointType_KneeRight, JointType_AnkleRight, JointType_AnkleRight, JointType_FootRight };

static const std::string skel_vertex_shader("skel_vs.glsl");
static const std::string skel_fragment_shader("skel_fs.glsl");
GLuint skel_shader_program = -1;

//Texture files and IDs

static const std::string cube_name = "cubemap";
GLuint cubemap_id = -1; //Texture id for cubemap

						//Mesh files and IDs

//Cube files and IDs
static const std::string cube_vs("cube_vs.glsl");
static const std::string cube_fs("cube_fs.glsl");
GLuint cube_shader_program = -1;
GLuint cube_vao = -1;
bool cube_enabled = false;

void glError()
{
	GLenum errCode;
	const GLubyte *errString;
	if ((errCode = glGetError()) != GL_NO_ERROR)
	{
		errString = gluErrorString(errCode);
		std::cout << "OpenGL Error: " << errString << std::endl;
	}
}

void initializeKinect()
{
	GetDefaultKinectSensor(&sensor);
	if (sensor)
	{
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Body,
			&reader);
	}
}

void getBodyData(IMultiSourceFrame * frame)
{
	IBodyFrameReference * bodyFrameReference = NULL;
	IBodyFrame * bodyFrame = NULL;

	frame->get_BodyFrameReference(&bodyFrameReference);
	bodyFrameReference->AcquireFrame(&bodyFrame);
	if (bodyFrameReference) bodyFrameReference->Release();

	if (!bodyFrame) return;

	IBody * body[BODY_COUNT] = { 0 };
	bodyFrame->GetAndRefreshBodyData(BODY_COUNT, body);
	for (int i = 0; i < BODY_COUNT; ++i)
	{
		body[i]->get_IsTracked(&bodyTracked);
		if (bodyTracked)
		{
			body[i]->GetJoints(JointType_Count, joints);
			break;
		}
	}

	if (bodyFrame) bodyFrame->Release();
}

void getKinectData()
{
	IMultiSourceFrame * frame = NULL;
	if (SUCCEEDED(reader->AcquireLatestFrame(&frame)))
	{
		getBodyData(frame);
	}
	if (frame) frame->Release();
}

inline static void glFixedPipelineLine(const glm::vec3 & pointA, const glm::vec3 & pointB)
{
	glVertex3fv(glm::value_ptr(pointA)); glVertex3fv(glm::value_ptr(pointB));
}

inline static glm::vec3 KinectCameraSpacePositionToglm(const CameraSpacePoint & point)
{
	return glm::vec3(point.X, point.Y, point.Z);
}


float camangle = 0.0f;
glm::vec3 campos(0.0f, 1.0f, 2.0f);
float aspect = 1.0f;

void draw_gui()
{
	glUseProgram(cube_shader_program);
	static bool first_frame = true;
	ImGui_ImplGlut_NewFrame();
	static bool show_window = true;

	ImGui::Begin("VAO Demo", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
	ImGui::Checkbox("Draw cube", &cube_enabled); ImGui::SameLine();
	ImGui::SliderFloat("View angle", &camangle, -180.0f, +180.0f);
	ImGui::SliderFloat3("Cam Pos", &campos[0], -20.0f, +20.0f);

	ImGui::End();

	ImGui::Render();
	first_frame = false;
}



void draw_cube(const glm::mat4& P, const glm::mat4& V)
{
	glUseProgram(cube_shader_program);
	int PVM_loc = glGetUniformLocation(cube_shader_program, "PVM");
	if (PVM_loc != -1)
	{
		glm::mat4 Msky = glm::scale(glm::vec3(50.0f));
		glm::mat4 PVM = P*V*Msky;
		PVM[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
		glUniformMatrix4fv(PVM_loc, 1, false, glm::value_ptr(PVM));
	}

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_CUBE_MAP, cubemap_id);
	int cube_loc = glGetUniformLocation(cube_shader_program, "cubemap");
	if (cube_loc != -1)
	{
		glUniform1i(cube_loc, 0); // we bound our texture to texture unit 1
	}

	draw_cube_vao(cube_vao);
}

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

void DrawIndexedSkeletons(const glm::mat4& P, const glm::mat4& V)
{
	glLineWidth(5.0f);
	glUseProgram(skel_shader_program);
	glm::mat4 PVM = P*V;

	int PVM_loc = glGetUniformLocation(skel_shader_program, "PVM");
	if (PVM_loc != -1)
	{
		glUniformMatrix4fv(PVM_loc, 1, false, glm::value_ptr(PVM));
	}

	glBindBuffer(GL_ARRAY_BUFFER, SkelVertsVBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, SkelIndex);

	// get a reference to an attrib variable name in a shader
	GLint pos_loc = glGetAttribLocation(skel_shader_program, "pos_attrib");
	glEnableVertexAttribArray(pos_loc);
	glVertexAttribPointer(pos_loc, 3, GL_FLOAT, false, 0, BUFFER_OFFSET(0));
	glDrawElements(GL_LINES, skelIndices.size(), GL_UNSIGNED_INT, BUFFER_OFFSET(0));

	glDisableVertexAttribArray(pos_loc);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// glut display callback function.
// This function gets called every time the scene gets redisplayed 
void display()
{
	static float frame = 0.0f;
	static int frame_id = 0;
	frame += 1.0f / 30.0f;
	float scale = 1.0f;
	frame_id++;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //Clear the back buffer

	glm::mat4 V = glm::lookAt(campos, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f))*glm::rotate(camangle, glm::vec3(0.0f, -1.0f, 0.0f));
	glm::mat4 P = glm::perspective(80.0f, aspect, 0.1f, 100.0f); //not affine

	if (cube_enabled)
	{
		draw_cube(P, V);
		glBindVertexArray(0);
	}

	
	if (bodyTracked)
	{
		//copy next frame into skelBuffer
		std::rotate(skelBuffer.begin(), skelBuffer.begin() + skelBuffer.size() - JointType_Count, skelBuffer.end());

		glm::mat4 M = glm::scale(glm::vec3(1.0f));

		float s = 1.1f;
		glm::mat4 T0 = glm::translate(-KinectCameraSpacePositionToglm(joints[JointType_SpineMid].Position));
		glm::mat4 T1 = glm::translate(KinectCameraSpacePositionToglm(joints[JointType_SpineMid].Position));

		for (int i = 0; i<JointType_Count; i++)
		{
			glm::vec4 p = glm::vec4(KinectCameraSpacePositionToglm(joints[i].Position), 1.0f);
			glm::vec4 p2 = M*p;
			skelBuffer[i] = glm::vec3(p2);
		}
		//updateNormals(); //not needed - computed in shader

		glBindBuffer(GL_ARRAY_BUFFER, SkelVertsVBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, skelBuffer.size() * 3 * sizeof(float), skelBuffer.data());

		//glBindBuffer(GL_ARRAY_BUFFER, SkelNormalsVBO);
		//glBufferSubData(GL_ARRAY_BUFFER, 0, skelNormal.size()*3*sizeof(float), skelNormal.data());

		glError();
	}

	const bool drawSkeletons = true;
	const bool drawCurves = true;
	
	if (drawSkeletons == true)
	{
		DrawIndexedSkeletons(P, V);
	}

	
	glError();
	//glUseProgram(0);
	draw_gui();
	glutSwapBuffers();
}

void idle()
{
	getKinectData();
	glutPostRedisplay();

	const int time_ms = glutGet(GLUT_ELAPSED_TIME);
	float time_sec = 0.001f*time_ms;

	/*glUseProgram(cube_shader_program);
	int time_loc = glGetUniformLocation(cube_shader_program, "time");
	if (time_loc != -1)
	{
		//double check that you are using glUniform1f
		glUniform1f(time_loc, time_sec);
	}

	glUseProgram(cube_shader_program);

	*/

}

void printGlInfo()
{
	std::cout << "Vendor: " << glGetString(GL_VENDOR) << std::endl;
	std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;
	std::cout << "Version: " << glGetString(GL_VERSION) << std::endl;
	std::cout << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
}

void initOpenGl()
{
	//glewInit();

	ImGui_ImplGlut_Init();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SPRITE);       // allows textured points
	glEnable(GL_PROGRAM_POINT_SIZE); //allows us to set point size in vertex shader
	glClearColor(0.35f, 0.35f, 0.35f, 0.0f);

	cubemap_id = LoadCube(cube_name);
	cube_shader_program = InitShader(cube_vs.c_str(), cube_fs.c_str());
	cube_vao = create_cube_vao();
	
	//////////////////////////////////////////////////////
	skelBuffer.resize(nFrames*JointType_Count);
	skelNormal.resize(skelBuffer.size());

	skelIndices.resize(nFrames * 40);

	int max_units;
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max_units);

	glGenBuffers(1, &SkelVertsVBO);
	glBindBuffer(GL_ARRAY_BUFFER, SkelVertsVBO);
	glBufferData(GL_ARRAY_BUFFER, skelBuffer.size() * 3 * sizeof(float), skelBuffer.data(), GL_STREAM_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	int p = 0;
	for (int f = 0; f<nFrames; f++)
	{
		for (int i = 0; i<40; i++)
		{
			skelIndices[p++] = skel[i] + JointType_Count*f;
		}
	}

	glGenBuffers(1, &SkelNormalsVBO);
	glBindBuffer(GL_ARRAY_BUFFER, SkelNormalsVBO);
	glBufferData(GL_ARRAY_BUFFER, skelNormal.size() * 3 * sizeof(float), skelNormal.data(), GL_STREAM_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);


	glGenBuffers(1, &SkelIndex);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, SkelIndex);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, skelIndices.size() * sizeof(unsigned int), skelIndices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	/////////////////////////////////////////////////////////

	/*glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(
		0.0, 0.0, 0.0,
		0.0, 0.0, 5.0,
		0.0, 1.0, 0.0
	); */
	skel_shader_program = InitShader(skel_vertex_shader.c_str(), skel_fragment_shader.c_str());
	glError();

}

// glut callbacks need to send keyboard and mouse events to imgui
void keyboard(unsigned char key, int x, int y)
{
	ImGui_ImplGlut_KeyCallback(key);
	//std::cout << "key : " << key << ", x: " << x << ", y: " << y << std::endl;
}

void keyboard_up(unsigned char key, int x, int y)
{
	ImGui_ImplGlut_KeyUpCallback(key);
}

void special_up(int key, int x, int y)
{
	ImGui_ImplGlut_SpecialUpCallback(key);
}

void passive(int x, int y)
{
	ImGui_ImplGlut_PassiveMouseMotionCallback(x, y);
}

void special(int key, int x, int y)
{
	ImGui_ImplGlut_SpecialCallback(key);
}

void motion(int x, int y)
{
	ImGui_ImplGlut_MouseMotionCallback(x, y);
}

void mouse(int button, int state, int x, int y)
{
	ImGui_ImplGlut_MouseButtonCallback(button, state);
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	aspect = (float)w / h;
}

int main(int argc, char **argv)
{
	initializeKinect();
	//Configure initial window state
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(5, 5);
	glutInitWindowSize(kinectDepthWidth, kinectDepthHeight);
	int win = glutCreateWindow("Bodygraph Demo");

	printGlInfo();

	/////////////////////////////////////////////////////
	glewInit();
	
	//Register callback functions with glut. 
	glutDisplayFunc(display);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(special);
	glutKeyboardUpFunc(keyboard_up);
	glutSpecialUpFunc(special_up);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutPassiveMotionFunc(motion);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);

	initOpenGl();
						   //Enter the glut event loop.
	glutMainLoop();
	glutDestroyWindow(win);
	return 0;
}

// where is the glPosition in vertex shaders variable defined???