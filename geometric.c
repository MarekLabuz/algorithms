#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

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
	struct Face F[150];
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

double distEF(struct Edge e, struct Face f) {
    double distances[5];

    struct Edge e1;
    struct Edge e2;
    struct Edge e3;
    e1.v1 = f.v1; e1.v2 = f.v2;
    e2.v1 = f.v2; e2.v2 = f.v3;
    e3.v1 = f.v1; e3.v2 = f.v3;

    distances[0] = distEE(e, e1);
    distances[1] = distEE(e, e2);
    distances[2] = distEE(e, e3);

    distances[3] = distPF(e.v1, f);
    distances[4] = distPF(e.v2, f);

    double min = distances[0];

    for(int i = 1; i < 5; i += 1) {
        if (distances[i] < min) {
            min = distances[i];
        }
    }

    return min;
}

double distFF(struct Face f1, struct Face f2) {
    double distances[6];

    struct Edge e1;
    struct Edge e2;
    struct Edge e3;
    e1.v1 = f1.v1; e1.v2 = f1.v2;
    e2.v1 = f1.v2; e2.v2 = f1.v3;
    e3.v1 = f1.v1; e3.v2 = f1.v3;

    distances[0] = distEF(e1, f2);
    distances[1] = distEF(e2, f2);
    distances[2] = distEF(e3, f2);

    e1.v1 = f2.v1; e1.v2 = f2.v2;
    e2.v1 = f2.v2; e2.v2 = f2.v3;
    e3.v1 = f2.v1; e3.v2 = f2.v3;

    distances[3] = distEF(e1, f1);
    distances[4] = distEF(e2, f1);
    distances[5] = distEF(e3, f1);

    double min = distances[0];

    for(int i = 1; i < 6; i += 1) {
        if (distances[i] < min) {
            min = distances[i];
        }
    }

    return min;
}

double distSS(struct Solid s1, struct Solid s2) {
    double min = distFF(s1.F[0], s2.F[0]);

    for (int i = 0; i < s1.size; i += 1) {
        for (int j = 0; j < s2.size; j += 1) {
            double distance = distFF(s1.F[i], s2.F[j]);
            if (distance < min) {
                min = distance;
            }
        }
    }

    return min;
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

    struct Edge e3;
    struct Point p5;
    p5.x = 10.0; p5.y = 100.0; p5.z = -60.0;
    e3.v1 = p4; e3.v2 = p5;

    printf("distEF %f\n", distEF(e3, f1));

    printf("distFF %f\n", distFF(f1, f1));

    // 2a
    struct Point _p1;
    struct Point _p2;
    struct Point _p3;
    _p1.x = 1.0; _p1.x = 0.0; _p1.x = 0.0;
    _p2.x = 0.0; _p2.x = 1.0; _p2.x = 0.0;
    _p3.x = 0.0; _p3.x = 0.0; _p3.x = 0.0;

    struct Point _p4;
    struct Point _p5;
    struct Point _p6;
    _p4.x = 100.0; _p4.x = 0.0; _p4.x = 0.13;
    _p5.x = 0.0; _p5.x = 100.0; _p5.x = 0.13;
    _p6.x = -100.0; _p6.x = -100.0; _p6.x = 0.13;

    struct Face _f1;
    _f1.v1 = _p1; _f1.v2 = _p2; _f1.v3 = _p3;
    struct Face _f2;
    _f2.v1 = _p4; _f2.v2 = _p5; _f2.v3 = _p6;

    printf("2a) distFF %f\n", distFF(_f1, _f2));

    // 2b
    struct Solid solids[2];
    solids[0].size = 132;
//    solids[0].F =
//    solids[1].size = 60;
    solids[1].size = 60;

    int solidIndex = -1;
    int faceIndex = -1;

    struct Point p_1;
    struct Point p_2;
    struct Point p_3;

    FILE *fp;
    char buff[255];
    char *array[3];

    fp = fopen("./solid_data.txt", "r");

    for(int s = 0; s < 2; s += 1) {
        fgets(buff, 255, (FILE*)fp);
        for (int i = 0; i < solids[s].size * 4; i += 4) {
            struct Point points[3];

            for (int j = 0; j < 3; j += 1) {
                fgets(buff, 255, (FILE*)fp);

//                printf("\nPoint: %s", buff);
                array[0] = strtok(buff, ";");
                array[1] = strtok(NULL, ";");
                array[2] = strtok(NULL, ";");

                points[j].x = strtod(array[0], NULL);
                points[j].y = strtod(array[1], NULL);
                points[j].z = strtod(array[2], NULL);
//                printf("%f\n", strtod(array[0], NULL));
//                printf("%f\n", strtod(array[1], NULL));
//                printf("%f\n", strtod(array[2], NULL));
            }
            fgets(buff, 255, (FILE*)fp);

            struct Face f;
            f.v1 = points[0];
            f.v2 = points[1];
            f.v3 = points[2];

            solids[s].F[i / 4] = f;
        }
    }

    fclose(fp);
    printf("2b) distSS %f\n", distSS(solids[0], solids[1]));
    return 0;
}