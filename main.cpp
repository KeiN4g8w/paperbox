/*
** 三角形分割された Alias OBJ 形式の形状データを
** OpenGL/GLUT を使ってアニメーション表示するプログラム
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <Eigen/core>
#include <Eigen/LU>
#include <Eigen/Dense>
#define EIGEN_NO_DEBUG // コード内のassertを無効化．
#define EIGEN_DONT_VECTORIZE // SIMDを無効化．
#define EIGEN_DONT_PARALLELIZE // 並列を無効化．
#define EIGEN_MPL2_ONLY // LGPLライセンスのコードを使わない．
using namespace Eigen;
#if defined(WIN32)
#  pragma warning(disable:4819)
#  pragma warning(disable:4996)
//#  pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")
#  include "freeglut.h"
#elif defined(X11)
#  include <GL/glut.h>
#elif defined(__APPLE__)
#  include <GLUT/glut.h>
#else
#  error "This platform is not supported."
#endif

/* 点の重さとばね定数 */
static GLfloat k = 0.01f;

/* 頂点の最大数と三角形の最大数 */
#define MAXPOINTS 8000
#define MAXFACES  8000

/* データファイル名 */
static const char data[] = "cube.obj";

/* 頂点の数と三角形の数 */
static int nv = 0, nf = 0;

/* 頂点データと三角形データ */
static GLfloat position[MAXPOINTS][3];
static GLfloat normal[MAXPOINTS][3];
static GLuint face[MAXFACES][3];
static GLfloat fnormal[MAXFACES][3];
static GLfloat cposition[MAXPOINTS][3];
static GLfloat force[MAXPOINTS][3];
static GLfloat pforce[MAXPOINTS][3];
static GLfloat b = -(1.0f);

/* 速度と加速度 */
static GLfloat vx[MAXPOINTS][3];
static GLfloat v0[MAXPOINTS][3];
static GLfloat va[MAXPOINTS][3];
static GLfloat smx[MAXPOINTS][3];

/* アニメーション */
static int animation = 0;

/* 経過時間 */
static GLfloat start;

/*
** ベクトルの内積
*/
static GLfloat dot(const GLfloat* v1, const GLfloat* v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/*
** ベクトルの長さ
*/
static GLfloat length(const GLfloat* v)
{
	if (dot(v, v) != 0.0f) {
		return sqrtf(dot(v, v));
	}
	else {
		return 0.0f;
	}
}

/*
** ベクトルの外積
*/
void cross(GLfloat* v, const GLfloat* v0, const GLfloat* v1, const GLfloat* v2)
{
	GLfloat dx1 = v1[0] - v0[0];
	GLfloat dy1 = v1[1] - v0[1];
	GLfloat dz1 = v1[2] - v0[2];
	GLfloat dx2 = v2[0] - v0[0];
	GLfloat dy2 = v2[1] - v0[1];
	GLfloat dz2 = v2[2] - v0[2];

	v[0] = dy1 * dz2 - dz1 * dy2;
	v[1] = dz1 * dx2 - dx1 * dz2;
	v[2] = dx1 * dy2 - dy1 * dx2;
}

/*
** ベクトルと転置したベクトルを掛けて行列に（３，３予定）
*/
static Matrix3f mkentry(GLfloat* v1, GLfloat* v2)
{
	Matrix3f m;

	m << v1[0] * v2[0], v1[0] * v2[1], v1[0] * v2[2],
		v1[1] * v2[0], v1[1] * v2[1], v1[1] * v2[2],
		v1[2] * v2[0], v1[2] * v2[1], v1[2] * v2[2];

	return m;
}

/*
** データファイルの読み込み
*/
static int loadFile(const char* name)
{
	FILE* fp = fopen(name, "r");
	char buf[1024];

	if (fp == NULL)
	{
		perror(name);
		return 1;
	}

	while (fgets(buf, sizeof buf, fp))
	{
		if (buf[0] == 'v' && buf[1] == ' ')
		{
			if (nv >= MAXPOINTS)
			{
				fprintf(stderr, "Too many vertices.\n");
				return 1;
			}
			if (sscanf(buf, "%*s %f %f %f", &position[nv][0], &position[nv][1], &position[nv][2]) != 3)
			{
				fclose(fp);
				fprintf(stderr, "Data format error.\n");
				return 1;
			}
			++nv;
		}
		else if (buf[0] == 'f' && buf[1] == ' ')
		{
			char s0[32], s1[32], s2[32];

			if (nf >= MAXFACES)
			{
				fprintf(stderr, "Too many faces.\n");
				return 1;
			}
			if (sscanf(buf + 2, "%31s %31s %31s", s0, s1, s2) != 3)
			{
				fprintf(stderr, "Can't read face data.\n");
				return 1;
			}
			face[nf][0] = atoi(s0) - 1;
			face[nf][1] = atoi(s1) - 1;
			face[nf][2] = atoi(s2) - 1;
			++nf;
		}
	}

	for (int i = 0; i < nv; i++) {
		position[i][0] *= 0.35f;
		position[i][1] *= 0.35f;
		position[i][2] *= 0.35f;
		cposition[i][0] = position[i][0];
		cposition[i][1] = position[i][1];
		cposition[i][2] = position[i][2];
	}

	for (int i = 0; i < nv; ++i) {
		vx[i][0] = 1.0f;
		vx[i][1] = 1.0f;
		vx[i][2] = 1.0f;
		v0[i][0] = 0.0f;
		v0[i][1] = 0.0f;
		v0[i][2] = 0.0f;
		va[i][0] = 0.0f;
		va[i][1] = 0.0f;
		va[i][2] = 0.0f;
	}

	return 0;
}

/*
** 面法線ベクトルの算出
*/
static void faceNormal(GLfloat fn[][3], GLfloat p[][3], GLuint f[][3], int nf)
{
	int i;

	for (i = 0; i < nf; ++i)
	{
		int v0 = f[i][0], v1 = f[i][1], v2 = f[i][2];

		cross(fn[i], p[v0], p[v1], p[v2]);
	}
}

/*
** 頂点の仮想法線ベクトルの算出
*/
static void vertexNormal(GLfloat vn[][3], int nv, GLfloat fn[][3], GLuint f[][3], int nf)
{
	int i;

	for (i = 0; i < nv; ++i)
	{
		vn[i][0] = vn[i][1] = vn[i][2] = 0.0f;
	}

	for (i = 0; i < nf; ++i)
	{
		int v0 = f[i][0], v1 = f[i][1], v2 = f[i][2];

		vn[v0][0] += fn[i][0];
		vn[v0][1] += fn[i][1];
		vn[v0][2] += fn[i][2];

		vn[v1][0] += fn[i][0];
		vn[v1][1] += fn[i][1];
		vn[v1][2] += fn[i][2];

		vn[v2][0] += fn[i][0];
		vn[v2][1] += fn[i][1];
		vn[v2][2] += fn[i][2];
	}

	for (i = 0; i < nv; ++i)
	{
		GLfloat a = length(vn[i]);

		if (a != 0.0f)
		{
			vn[i][0] /= a;
			vn[i][1] /= a;
			vn[i][2] /= a;
		}
	}
}

static void vertex(GLfloat vn[][3], int nv, GLfloat fn[][3], GLuint f[][3], int nf)
{
	int i;

	for (i = 0; i < nv; ++i)
	{
		vn[i][0] = vn[i][1] = vn[i][2] = 0.0f;
	}

	for (i = 0; i < nf; ++i)
	{
		int v0 = f[i][0], v1 = f[i][1], v2 = f[i][2];

		vn[v0][0] += fn[v0][0];
		vn[v0][1] += fn[v0][1];
		vn[v0][2] += fn[v0][2];

		vn[v1][0] += fn[v1][0];
		vn[v1][1] += fn[v1][1];
		vn[v1][2] += fn[v1][2];

		vn[v2][0] += fn[v2][0];
		vn[v2][1] += fn[v2][1];
		vn[v2][2] += fn[v2][2];
	}
}

/*
** ばね
*/
static void spring(GLfloat force[][3], GLfloat cp[][3], GLfloat p[][3], GLuint f[][3], int nf)
{
	GLfloat cpos[3000][3], pos[3000][3], fo[3000][3];

	for (int i = 0; i < 3000; ++i) {
		for (int j = 0; j < 3; ++j) {
			cpos[i][j] = cp[i][j];
			pos[i][j] = p[i][j];
			fo[i][j] = force[i][j];
		}
	}

	for (int i = 0; i < nf; ++i)
	{
		GLfloat l[3], x[3];

		l[0] = 0.0f;
		l[1] = 0.0f;
		l[2] = 0.0f;

		x[0] = 0.0f;
		x[1] = 0.0f;
		x[2] = 0.0f;

		int v0 = f[i][0], v1 = f[i][1], v2 = f[i][2];
		GLfloat a[3][3], b[3][3], c[3][3];

		a[0][0] = pos[v0][0] - pos[v1][0];
		a[0][1] = pos[v0][1] - pos[v1][1];
		a[0][2] = pos[v0][2] - pos[v1][2];
		a[1][0] = pos[v1][0] - pos[v2][0];
		a[1][1] = pos[v1][1] - pos[v2][1];
		a[1][2] = pos[v1][2] - pos[v2][2];
		a[2][0] = pos[v2][0] - pos[v0][0];
		a[2][1] = pos[v2][1] - pos[v0][1];
		a[2][2] = pos[v2][2] - pos[v0][2];

		b[0][0] = cpos[v0][0] - cpos[v1][0];
		b[0][1] = cpos[v0][1] - cpos[v1][1];
		b[0][2] = cpos[v0][2] - cpos[v1][2];
		b[1][0] = cpos[v1][0] - cpos[v2][0];
		b[1][1] = cpos[v1][1] - cpos[v2][1];
		b[1][2] = cpos[v1][2] - cpos[v2][2];
		b[2][0] = cpos[v2][0] - cpos[v0][0];
		b[2][1] = cpos[v2][1] - cpos[v0][1];
		b[2][2] = cpos[v2][2] - cpos[v0][2];

		c[0][0] = a[0][0] - b[0][0];
		c[0][1] = a[0][1] - b[0][1];
		c[0][2] = a[0][2] - b[0][2];
		c[1][0] = a[1][0] - b[1][0];
		c[1][1] = a[1][1] - b[1][1];
		c[1][2] = a[1][2] - b[1][2];
		c[2][0] = a[2][0] - b[2][0];
		c[2][1] = a[2][1] - b[2][1];
		c[2][2] = a[2][2] - b[2][2];

		l[0] = length(a[0]);
		l[1] = length(a[1]);
		l[2] = length(a[2]);

		x[0] = length(b[0]);
		x[1] = length(b[1]);
		x[2] = length(b[2]);



		fo[v0][0] -= (c[0][0] * k * (l[0] - x[0]) - c[2][0] * k * (l[2] - x[2]));
		fo[v0][1] -= (c[0][1] * k * (l[0] - x[0]) - c[2][1] * k * (l[2] - x[2]));
		fo[v0][2] -= (c[0][2] * k * (l[0] - x[0]) - c[2][2] * k * (l[2] - x[2]));
		fo[v1][0] -= (c[1][0] * k * (l[1] - x[1]) - c[0][0] * k * (l[0] - x[0]));
		fo[v1][1] -= (c[1][1] * k * (l[1] - x[1]) - c[0][1] * k * (l[0] - x[0]));
		fo[v1][2] -= (c[1][2] * k * (l[1] - x[1]) - c[0][2] * k * (l[0] - x[0]));
		fo[v2][0] -= (c[2][0] * k * (l[2] - x[2]) - c[1][0] * k * (l[1] - x[1]));
		fo[v2][1] -= (c[2][1] * k * (l[2] - x[2]) - c[1][1] * k * (l[1] - x[1]));
		fo[v2][2] -= (c[2][2] * k * (l[2] - x[2]) - c[1][2] * k * (l[1] - x[1]));
	}

	for (int i = 0; i < 3000; ++i) {
		for (int j = 0; j < 3; ++j) {
			force[i][j] = fo[i][j];
		}
	}
	/*
	for (int i = 0; i < 3; ++i)
	{
		if ((p[i] - cp[i] < 0.000001f && p[i] - cp[i] > 0.000000f) || (p[i] - cp[i] < 0.000000f && p[i] - cp[i] > -0.000001f)) {
			e[i] = 0.0f;
		}
		else {
			if ((v[i] < 0.000000f || v[i] > 0.000000f) && (l < 0.000000f || l > 0.000000f)) {
				e[i] = v[i] / l;
			}
			else {
				e[i] = 0.0f;
			}

		}
	}
	*/
}

/*
** シェイプマッチング
*/
static void shapematching(GLfloat smx[][3], GLfloat cp[][3], GLfloat p[][3], GLuint f[][3], int nf)
{
	GLfloat cpos[5000][3], pos[5000][3];

	for (int i = 0; i < 5000; ++i) {
		for (int j = 0; j < 3; ++j) {
			cpos[i][j] = cp[i][j];
			pos[i][j] = p[i][j];
		}
	}

	for (int i = 0; i < nf; ++i)
	{
		int v0 = f[i][0], v1 = f[i][1], v2 = f[i][2];
		GLfloat c = 1.0f;

		GLfloat cg[3], g[3], qi[3][3], pi[3][3], d[3];
		cg[0] = (cpos[v0][0] + cpos[v1][0] + cpos[v2][0]) * 0.33f;
		cg[1] = (cpos[v0][1] + cpos[v1][1] + cpos[v2][1]) * 0.33f;
		cg[2] = (cpos[v0][2] + cpos[v1][2] + cpos[v2][2]) * 0.33f;
		g[0] = (pos[v0][0] + pos[v1][0] + pos[v2][0]) * 0.33f;
		g[1] = (pos[v0][1] + pos[v1][1] + pos[v2][1]) * 0.33f;
		g[2] = (pos[v0][2] + pos[v1][2] + pos[v2][2]) * 0.33f;

		for (int j = 0; j < 3; ++j) {
			qi[0][j] = cpos[v0][j] - cg[j];
			qi[1][j] = cpos[v1][j] - cg[j];
			qi[2][j] = cpos[v2][j] - cg[j];
			pi[0][j] = pos[v0][j] - g[j];
			pi[1][j] = pos[v1][j] - g[j];
			pi[2][j] = pos[v2][j] - g[j];
		}

		Matrix3f P1, P2, P3, Q1, Q2, Q3, P, Q, Q_1, A, AT, S, S2, S_1, R, D2, D, V, V_1;
		//P = Matrix3f::Zero(3, 3);
		//Q = Matrix3f::Zero(3, 3);
		//A = Matrix3f::Zero(3, 3);
		//S = Matrix3f::Zero(3, 3);
		//R = Matrix3f::Zero(3, 3);

		P1 = mkentry(pi[0], qi[0]);
		P2 = mkentry(pi[1], qi[1]);
		P3 = mkentry(pi[2], qi[2]);
		Q1 = mkentry(qi[0], qi[0]);
		Q2 = mkentry(qi[1], qi[1]);
		Q3 = mkentry(qi[2], qi[2]);

		P = P1 + P2 + P3;
		Q = Q1 + Q2 + Q3;
		Q_1 = Q.inverse();
		A = P * Q_1;
		AT = A.transpose();
		S2 = A * AT;
		SelfAdjointEigenSolver<Matrix3f> es(S2);
		Map<Vector3f>(&d[0], 3) = es.eigenvalues();
		D2.coeffRef(0, 0) = d[0];
		D2.coeffRef(0, 1) = 0.0f;
		D2.coeffRef(0, 2) = 0.0f;
		D2.coeffRef(1, 0) = 0.0f;
		D2.coeffRef(1, 1) = d[1];
		D2.coeffRef(1, 2) = 0.0f;
		D2.coeffRef(2, 0) = 0.0f;
		D2.coeffRef(2, 1) = 0.0f;
		D2.coeffRef(2, 2) = d[2];
		D = D2.array().sqrt();
		V = es.eigenvectors();
		V_1 = V.inverse();
		S = V * D * V_1;
		//FullPivLU< Matrix3f > lu2(S);
		S_1 = S.inverse();
		R = A * S_1;

		GLfloat r[3][3];
		Map<Matrix3f>(&r[0][0], 3, 3) = R;

		pos[v0][0] = r[0][0] * (cpos[v0][0] - cg[0]) + r[0][1] * (cpos[v0][1] - cg[1]) + r[0][2] * (cpos[v0][2] - cg[2]) + g[0];
		pos[v0][1] = r[1][0] * (cpos[v0][0] - cg[0]) + r[1][1] * (cpos[v0][1] - cg[1]) + r[1][2] * (cpos[v0][2] - cg[2]) + g[1];
		pos[v0][2] = r[2][0] * (cpos[v0][0] - cg[0]) + r[2][1] * (cpos[v0][1] - cg[1]) + r[2][2] * (cpos[v0][2] - cg[2]) + g[2];
		pos[v1][0] = r[0][0] * (cpos[v1][0] - cg[0]) + r[0][1] * (cpos[v1][1] - cg[1]) + r[0][2] * (cpos[v1][2] - cg[2]) + g[0];
		pos[v1][1] = r[1][0] * (cpos[v1][0] - cg[0]) + r[1][1] * (cpos[v1][1] - cg[1]) + r[1][2] * (cpos[v1][2] - cg[2]) + g[1];
		pos[v1][2] = r[2][0] * (cpos[v1][0] - cg[0]) + r[2][1] * (cpos[v1][1] - cg[1]) + r[2][2] * (cpos[v1][2] - cg[2]) + g[2];
		pos[v2][0] = r[0][0] * (cpos[v2][0] - cg[0]) + r[0][1] * (cpos[v2][1] - cg[1]) + r[0][2] * (cpos[v2][2] - cg[2]) + g[0];
		pos[v2][1] = r[1][0] * (cpos[v2][0] - cg[0]) + r[1][1] * (cpos[v2][1] - cg[1]) + r[1][2] * (cpos[v2][2] - cg[2]) + g[1];
		pos[v2][2] = r[2][0] * (cpos[v2][0] - cg[0]) + r[2][1] * (cpos[v2][1] - cg[1]) + r[2][2] * (cpos[v2][2] - cg[2]) + g[2];
	}

	for (int i = 0; i < 5000; ++i) {
		smx[i][0] = pos[i][0] - cpos[i][0];
		smx[i][1] = pos[i][1] - cpos[i][1];
		smx[i][2] = pos[i][2] - cpos[i][2];
	}
}

/*
** 画面表示
*/
static void display(void)
{
	static const GLfloat bodyColor[] = { 0.9f, 0.9f, 0.9f, 1.0f };
	static const GLfloat lightPosition[] = { 1.0f, 0.8f, 0.6f, 0.0f };

	/* 画面クリア */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/* 視点の設定 */
	glLoadIdentity();
	gluLookAt(3.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	/* 光源位置の設定 */
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

	/* 車 */
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, bodyColor);
	glVertexPointer(3, GL_FLOAT, 0, position);
	glNormalPointer(GL_FLOAT, 0, normal);
	glDrawElements(GL_TRIANGLES, nf * 3, GL_UNSIGNED_INT, face);

	glutSwapBuffers();

	/* 頂点位置の更新 */
	if (animation)
	{
		//GLfloat elapsed_time = glutGet(GLUT_ELAPSED_TIME) - start;
		//elapsed_time = 0.001f;
		GLfloat f = 0.0001f;

		for (int i = 0; i < nv; ++i)
		{
			GLfloat x = vx[i][1] * f + va[i][1] * f * f;

			if (position[i][1] > b) {
				if (position[i][1] - x < b) {
					position[i][1] = b;
				}
				else {
					position[i][1] -= x;
				}
			}
		}

		shapematching(smx, cposition, position, face, nf);
		//vertex(pforce, nv, force, face, nf);
		spring(force, cposition, position, face, nf);

		for (int j = 0; j < nv; ++j)
		{
			position[j][0] += smx[j][0] + force[j][0] * f;
			position[j][1] += smx[j][1] + force[j][1] * f;
			position[j][2] += smx[j][2] + force[j][2] * f;

			va[j][0] = ((position[j][0] - cposition[j][0]) * 10000.0f - vx[j][0]) * 10000.0f;
			va[j][1] = ((position[j][1] - cposition[j][1]) * 10000.0f - vx[j][1]) * 10000.0f;
			va[j][2] = ((position[j][2] - cposition[j][2]) * 10000.0f - vx[j][2]) * 10000.0f;

			vx[j][0] = (position[j][0] - cposition[j][0]) * 10000.0f;
			vx[j][1] = (position[j][1] - cposition[j][1]) * 10000.0f;
			vx[j][2] = (position[j][2] - cposition[j][2]) * 10000.0f;

			cposition[j][0] = position[j][0];
			cposition[j][1] = position[j][1];
			cposition[j][2] = position[j][2];
		}

		/* 頂点を移動したので法線ベクトルを再計算する */
		faceNormal(fnormal, position, face, nf);
		vertexNormal(normal, nv, fnormal, face, nf);

		for (int i = 0; i < nv; ++i)
		{
			force[i][0] = 0.0f;
			force[i][1] = 0.0f;
			force[i][2] = 0.0f;
		}
	}
}

/*
** resise() でウィンドウの幅と高さを得る
** ウィンドウの座標系をビューポート（出力画像）と一致させる
*/
static void resize(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0, (double)w / (double)h, 1.0, 10.0);
	glMatrixMode(GL_MODELVIEW);
}

/*
** アニメーション
*/
static void idle(void)
{
	glutPostRedisplay();
}

/*
** キー操作
*/
static void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'q':
	case 'Q':
	case '\033':
		/* q, Q, ESC キーで終了する */
		exit(0);
	case 'g':
	case 'G':
		/* g, G キーでアニメーションを開始／停止する */
		animation = 1 - animation;
		break;
	default:
		break;
	}
}

/*
** 初期化
*/
static void init()
{
	loadFile(data);
	faceNormal(fnormal, position, face, nf);
	vertexNormal(normal, nv, fnormal, face, nf);
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

/*
** glut/OpenGL の初期化と実行
*/
int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitWindowSize(320, 240);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
#if !defined(__APPLE__)
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
	glutCreateWindow("CG sample");
	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	init();
	start = glutGet(GLUT_ELAPSED_TIME);
	glutMainLoop();
	return 0;
}

