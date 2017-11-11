#include <stdio.h>
#include <math.h>

struct Point {
	double x, y, z;
};

struct Face {
	struct Point v1, v2, v3;
};

struct Edge {
	struct Point v1, v2;
};

struct Solid {
	int size;
	struct Face F[];
};

double distPP(struct Point a, struct Point b) {
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
}

double dot(struct Point u, struct Point v) {
	return u.x * v.x + u.y * v.y + u.z * v.z;
}

double norm(struct Point v) {
	return sqrt(dot(v, v));
}

double d(struct Point u, struct Point v) {
	struct Point r;
	r.x = u.x - v.x;
	r.y = u.y - v.y;
	r.z = u.z - v.z;
	return norm(r);
}

struct Point getVectorPoint(struct Point p1, struct Point p2) {
	struct Point v;
	v.x = p2.x - p1.x;
	v.y = p2.y - p1.y;
	v.z = p2.z - p1.z;
	return v;
}

double distEP(struct Edge E, struct Point P) {
	struct Point v = getVectorPoint(E.v1, E.v2);
	// v.x = E.v2.x - E.v1.x;
	// v.y = E.v2.y - E.v1.y;
	// v.z = E.v2.z - E.v1.z;

	struct Point w = getVectorPoint(E.v1, P);

	double c1 = dot(w, v);
    double c2 = dot(v, v);
    double b = c1 / c2;

    struct Point Pb;
    Pb.x = E.v1.x + b * v.x;
    Pb.y = E.v1.y + b * v.y;
    Pb.z = E.v1.z + b * v.z;
    return d(P, Pb);
}

double distEE(struct Edge E1, struct Edge E2) {
	struct Point u = getVectorPoint(E1.v1, E1.v2);
	struct Point v = getVectorPoint(E2.v1, E2.v2);
	struct Point w = getVectorPoint(E2.v1, E1.v1);

	float a = dot(u,u);
    float b = dot(u,v);
    float c = dot(v,v);
    float d = dot(u,w);
    float e = dot(v,w);
    float D = a * c - b * b;
    float sc, tc;

    if (D <= 0) {
        sc = 0.0;
        tc = (b > c ? d / b : e / c);
    } else {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }

    struct Point dP;
    dP.x = w.x + (sc * u.x) - (tc * v.x);
    dP.y = w.y + (sc * u.y) - (tc * v.y);
    dP.z = w.z + (sc * u.z) - (tc * v.z);

	return norm(dP);
}

int main() {
    struct Point p1;
    struct Point p2;

    p1.x = 1.0; p1.y = 2.0; p1.z = 3.0;
    p2.x = 4.0; p2.y = 5.0; p2.z = 6.0;

    printf("%f\n", distPP(p1, p2));

    struct Edge e1;
    struct Point p3;

    p3.x = 40.0; p3.y = 5.0; p3.z = 6.0;
    e1.v1 = p1; e1.v2 = p2;

    printf("%f\n", distEP(e1, p3));

    struct Edge e2;
    struct Point p4;

    p4.x = 20.0; p4.y = 10.0; p4.z = -6.0;
    e2.v1 = p4; e2.v2 = p3;

    printf("%f\n", distEE(e1, e2));

    return 0;
}