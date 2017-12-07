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


float particles[3*1000];
std::vector<glm::vec3> startTime;

GLuint particlesVAO[2];
GLuint feedback[2]; // Transform feedback objects
GLuint partPosBuf[2];
GLuint partTimeBuf[2];
int drawBuf = 0;

// VBOs
GLuint SkelVertsVBO = -1;
GLuint SkelNormalsVBO = -1;
GLuint billboard_vertex_buffer = -1;
GLuint particles_position_buffer = -1;
GLuint particles_color_buffer = -1;

float camangle = 0.0f;
glm::vec3 campos(0.0f, 1.0f, 2.0f);
float aspect = 1.0f;

glm::mat4 M = glm::scale(glm::vec3(1.0f));
glm::mat4 V = glm::lookAt(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f));
glm::mat4 P = glm::perspective(80.0f, aspect, 0.1f, 100.0f); //not affine

float cam_height = 0.0f;
const int nBones = 20;
static const GLfloat g_vertex_buffer_data[] = {
	-0.5f, -0.5f, 0.0f,
	0.5f, -0.5f, 0.0f,
	-0.5f, 0.5f, 0.0f,
	0.5f, 0.5f, 0.0f,
};

static const std::string skel_vertex_shader("skel_vs.glsl");
static const std::string skel_fragment_shader("skel_fs.glsl");
static const std::string skel_geometry_shader("skel_gs.glsl");
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

//Locations
GLint part_pos_loc;

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




void draw_gui()
{
	static bool first_frame = true;
	ImGui_ImplGlut_NewFrame();
	static bool show_window = true;

	ImGui::Begin("VAO Demo", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
	//ImGui::Checkbox("Draw cube", &cube_enabled); ImGui::SameLine();
	ImGui::SliderFloat("View angle", &camangle, 0.0f, 0.0f);
	ImGui::SliderFloat3("Cam Pos", &campos[0], 0.0f, 0.0f);

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

void RenderPass1(GLuint functionLocation)
{
	//////////// Update pass ///////////////
	glError();
	glUniformSubroutinesuiv(GL_VERTEX_SHADER, 1, &functionLocation);
	glError();

	//glBindBuffer(GL_ARRAY_BUFFER, partPosBuf[1-drawBuf]);
	//glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER, sizeof(particles), NULL, GL_STATIC_READ);

	glError();
	glBindVertexArray(particlesVAO[1 - drawBuf]);
	//glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, partPosBuf[1 - drawBuf]);
	glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, partPosBuf[1 - drawBuf]);
	//glBindBuffer(GL_ARRAY_BUFFER, partPosBuf[drawBuf]);

	// Disable rendering
	glEnable(GL_RASTERIZER_DISCARD);

	// Bind the feedback object for the buffers to be drawn next
	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, feedback[drawBuf]);

	// Draw points from input buffer with transform feedback
	glBeginTransformFeedback(GL_POINTS);
	
	glDrawArrays(GL_POINTS, 0, 1000);
	
	glEndTransformFeedback();
	//glFlush();
	// Enable rendering
	glDisable(GL_RASTERIZER_DISCARD);

}

void RenderPass2( GLuint functionLocation)
{
	//////////// Render pass ///////////////
	glUniformSubroutinesuiv(GL_VERTEX_SHADER, 1, &functionLocation);
	glClear(GL_COLOR_BUFFER_BIT);

	// Un-bind the feedback object.
	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, 0);
	
	// Draw the sprites from the feedback buffer
	glBindVertexArray(particlesVAO[drawBuf]);
	glDrawArrays(GL_POINTS, 0, 1000);
	/*
	GLfloat feed[3000];
	// Update vertices' position and velocity using transform feedback
	glGetBufferSubData(GL_TRANSFORM_FEEDBACK_BUFFER, 0, sizeof(feed), feed);

	for (int i = 0; i < 3000-3; i=i+3) {
		particles[i] = feed[i];
		particles[i + 1] = feed[i + 1];
		particles[i + 2] = feed[i + 2];
	}
	*/

	/*if (feed != NULL) {
		for (int i = 0; i < 3000 - 3; i = i + 3) {
			std::cout << particles[i] << std::endl;
			std::cout << particles[i + 1] << std::endl;
			std::cout << particles[i + 2] << std::endl;
		}
	} */
	

	// glBufferSubData() updates an existing buffer.
	//glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(particles), particles);

	
}

#define BUFFER_OFFSET(i) ((char *)NULL + (i))

void DrawIndexedSkeletons(const glm::mat4& P, const glm::mat4& V)
{	
	glError();
	//glLineWidth(5.0f);
	glPointSize(5.0);
	glUseProgram(skel_shader_program);
	glm::mat4 PVM = P*V;

	int PVM_loc = glGetUniformLocation(skel_shader_program, "PVM");
	if (PVM_loc != -1)
	{
		glUniformMatrix4fv(PVM_loc, 1, false, glm::value_ptr(PVM));
	}
	//glError();
	//glBindVertexArray(particlesVAO[drawBuf]);
	//glError();
	//glBindBuffer(GL_ARRAY_BUFFER, partPosBuf[drawBuf]);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(particles) * sizeof(float), particles, GL_STREAM_DRAW);

	//// get a reference to an attrib variable name in a shader
	//part_pos_loc = glGetAttribLocation(skel_shader_program, "curpos");
	//glEnableVertexAttribArray(part_pos_loc);
	//glVertexAttribPointer(part_pos_loc, 3, GL_FLOAT, GL_FALSE, 0,NULL);
	//glDrawArrays(GL_POINTS, 0, particles.size() * 3);

	GLuint updateSub = glGetSubroutineIndex(skel_shader_program, GL_VERTEX_SHADER, "update");
	GLuint renderSub = glGetSubroutineIndex(skel_shader_program, GL_VERTEX_SHADER, "render");
	glError();
	RenderPass1(updateSub);
	glError();
	RenderPass2(renderSub);

	// Swap buffers
	drawBuf = 1 - drawBuf;

	glError();
}


// glut display callback function.
// This function gets called every time the scene gets redisplayed 
void display()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //Clear the back buffer

	
	if (cube_enabled)
	{
		//draw_cube(P, V);
		//glBindVertexArray(0);
	}


	if (bodyTracked)
	{
	
		glError();
	}

	const bool drawSkeletons = true;

	if (drawSkeletons == true)
	{
		DrawIndexedSkeletons(P, V);
	}

	glError();
	draw_gui();
	glutSwapBuffers();
}

void idle()
{
	getKinectData();
	glutPostRedisplay();

	const int time_ms = glutGet(GLUT_ELAPSED_TIME);
	float time_sec = 0.001f*time_ms;
	int time_loc = glGetUniformLocation(skel_shader_program, "time");
	if (time_loc != -1) {
		glUniform1f(time_loc, time_sec);
	}
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

	ImGui_ImplGlut_Init();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SPRITE);       // allows textured points
	glEnable(GL_PROGRAM_POINT_SIZE); //allows us to set point size in vertex shader
	glClearColor(0.35f, 0.35f, 0.35f, 0.0f);

	/*cubemap_id = LoadCube(cube_name);
	cube_shader_program = InitShader(cube_vs.c_str(), cube_fs.c_str());
	cube_vao = create_cube_vao(); */

	//////////////////////////////////////////////////////
	//fill particle array
	float  i = -1.0f;
	for (unsigned int k = 0; k < 3000-3; k=k+3) {
		i += .04f;
		particles[k] = i ;
		particles[k + 1] = 0.0f;
		particles[k + 2] = 0.0f;
	}

	int max_units;
	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &max_units);

	//Init Shader
	skel_shader_program = InitShader(skel_vertex_shader.c_str(), skel_fragment_shader.c_str());
	//Get Location
	part_pos_loc = glGetAttribLocation(skel_shader_program, "curpos");
	GLenum st;
/////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////
	///////////Create VAo and VBO HERE/////////////////

	glGenVertexArrays(1, &particlesVAO[0]);
	glBindVertexArray(particlesVAO[0]);
		glGenBuffers(1, &partPosBuf[0]);
		glBindBuffer(GL_ARRAY_BUFFER, partPosBuf[0]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(particles), nullptr, GL_STREAM_DRAW);	
		glEnableVertexAttribArray(part_pos_loc);
		glVertexAttribPointer(part_pos_loc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

	glGenVertexArrays(1, &particlesVAO[1]);
	glBindVertexArray(particlesVAO[1]);
		glGenBuffers(1, &partPosBuf[1]);
		glBindBuffer(GL_ARRAY_BUFFER, partPosBuf[1]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(particles), particles, GL_STREAM_DRAW);
		glEnableVertexAttribArray(part_pos_loc);
		glVertexAttribPointer(part_pos_loc, 3, GL_FLOAT, GL_FALSE, 0, NULL);

		glError();

	glBindVertexArray(0);

	// Setup the feedback objects
	glGenTransformFeedbacks(2, feedback);
	// Transform feedback 0
	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, feedback[0]);
	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, partPosBuf[0]);

	// Transform feedback 1
	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, feedback[1]);
	glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, partPosBuf[1]);

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