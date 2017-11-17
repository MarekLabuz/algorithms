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

double distLL(struct Edge E1, struct Edge E2) {
	struct Point u = getVectorPoint(E1.v1, E1.v2);
	struct Point v = getVectorPoint(E2.v1, E2.v2);
	struct Point w = getVectorPoint(E2.v1, E1.v1);

	float a = dot(u, u);
    float b = dot(u, v);
    float c = dot(v, v);
    float d = dot(u, w);
    float e = dot(v, w);
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

double distEE(struct Edge E1, struct Edge E2) {
    struct Point u = getVectorPoint(E1.v1, E1.v2);
	struct Point v = getVectorPoint(E2.v1, E2.v2);
	struct Point w = getVectorPoint(E2.v1, E1.v1);

    float a = dot(u, u);
    float b = dot(u, v);
    float c = dot(v, v);
    float d = dot(u, w);
    float e = dot(v, w);
    float D = a * c - b * b;
    float sc, sN, sD = D;
    float tc, tN, tD = D;

    if (D <= 0) {
        sN = 0.0;
        sD = 1.0;
        tN = e;
        tD = c;
    }
    else {
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        if (sN < 0.0) {
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {
        tN = 0.0;

        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {
        tN = tD;
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    sc = (fabsf(sN) <= 0 ? 0.0 : sN / sD);
    tc = (fabsf(tN) <= 0 ? 0.0 : tN / tD);

    struct Point dP;
    dP.x = w.x + (sc * u.x) - (tc * v.x);
    dP.y = w.y + (sc * u.y) - (tc * v.y);
    dP.z = w.z + (sc * u.z) - (tc * v.z);

    return norm(dP);
}

double distPF(struct Point P, struct Face F) {

    struct Point diff = getVectorPoint(F.v1, P);
    struct Point edge0 = getVectorPoint(F.v1, F.v2);
    struct Point edge1 = getVectorPoint(F.v1, F.v3);

    double a00 = dot(edge0, edge0);
    double a01 = dot(edge0, edge1);
    double a11 = dot(edge1, edge1);
    double b0 = -dot(diff, edge0);
    double b1 = -dot(diff, edge1);

    double const zero = 0.0;
    double const one = 1.0;
    double det = a00 * a11 - a01 * a01;
    double t0 = a01 * b1 - a11 * b0;
    double t1 = a01 * b0 - a00 * b1;

    if (t0 + t1 <= det) {
        if (t0 < zero) {
            if (t1 < zero) {
                if (b0 < zero) {
                    t1 = zero;
                    if (-b0 >= a00) {
                        t0 = one;
                    } else {
                        t0 = -b0 / a00;
                    }
                } else {
                    t0 = zero;
                    if (b1 >= zero) {
                        t1 = zero;
                    } else if (-b1 >= a11) {
                        t1 = one;
                    } else {
                        t1 = -b1 / a11;
                    }
                }
            } else {
                t0 = zero;
                if (b1 >= zero) {
                    t1 = zero;
                } else if (-b1 >= a11)  // V2
                {
                    t1 = one;
                } else {
                    t1 = -b1 / a11;
                }
            }
        }
        else if (t1 < zero) {
            t1 = zero;
            if (b0 >= zero) {
                t0 = zero;
            } else if (-b0 >= a00) {
                t0 = one;
            } else {
                t0 = -b0 / a00;
            }
        } else {
            double invDet = one / det;
            t0 *= invDet;
            t1 *= invDet;
        }
    } else {
        double tmp0, tmp1, numer, denom;

        if (t0 < zero) {
            tmp0 = a01 + b0;
            tmp1 = a11 + b1;
            if (tmp1 > tmp0) {
                numer = tmp1 - tmp0;
                denom = a00 - 2.0 * a01 + a11;
                if (numer >= denom) {
                    t0 = one;
                    t1 = zero;
                } else {
                    t0 = numer / denom;
                    t1 = one - t0;
                }
            } else {
                t0 = zero;
                if (tmp1 <= zero) {
                    t1 = one;
                } else if (b1 >= zero) {
                    t1 = zero;
                } else {
                    t1 = -b1 / a11;
                }
            }
        } else if (t1 < zero) {
            tmp0 = a01 + b1;
            tmp1 = a00 + b0;
            if (tmp1 > tmp0) {
                numer = tmp1 - tmp0;
                denom = a00 - 2.0 * a01 + a11;
                if (numer >= denom) {
                    t1 = one;
                    t0 = zero;
                } else {
                    t1 = numer / denom;
                    t0 = one - t1;
                }
            } else {
                t1 = zero;
                if (tmp1 <= zero) {
                    t0 = one;
                } else if (b0 >= zero) {
                    t0 = zero;
                } else {
                    t0 = -b0 / a00;
                }
            }
        } else {
            numer = a11 + b1 - a01 - b0;
            if (numer <= zero) {
                t0 = zero;
                t1 = one;
            } else {
                denom = a00 - 2.0 * a01 + a11;
                if (numer >= denom) {
                    t0 = one;
                    t1 = zero;
                } else {
                    t0 = numer / denom;
                    t1 = one - t0;
                }
            }
        }
    }

    struct Point closest;
    closest.x = F.v1.x + t0 * edge0.x + t1 * edge1.x;
    closest.y = F.v1.y + t0 * edge0.y + t1 * edge1.y;
    closest.z = F.v1.z + t0 * edge0.z + t1 * edge1.z;

    diff.x = P.x - closest.x;
    diff.y = P.y - closest.y;
    diff.z = P.z - closest.z;

    return norm(diff);
}

double distEF() {
    return 0;
}

int main() {
    struct Point p1;
    struct Point p2;

    p1.x = 1.0; p1.y = 2.0; p1.z = 3.0;
    p2.x = 4.0; p2.y = 5.0; p2.z = 6.0;

    printf("distPP %f\n", distPP(p1, p2));

    struct Edge e1;
    struct Point p3;

    p3.x = 40.0; p3.y = 5.0; p3.z = 6.0;
    e1.v1 = p1; e1.v2 = p2;

    printf("distEP %f\n", distEP(e1, p3));

    struct Edge e2;
    struct Point p4;

    p4.x = 20.0; p4.y = 10.0; p4.z = -6.0;
    e2.v1 = p4; e2.v2 = p2;

    printf("distEE %f\n", distEE(e1, e2));

    struct Face f1;
    f1.v1 = p1; f1.v2 = p2; f1.v3 = p3;

    printf("distPF %f\n", distPF(p4, f1));

    return 0;
}