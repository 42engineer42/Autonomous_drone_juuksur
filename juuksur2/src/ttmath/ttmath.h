#ifndef TTMATH_H
#define TTMATH_H

#include <math.h>
#include <assert.h>

#ifdef __cplusplus
#define restrict __restrict__
#endif


#define TT_CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

// type used for real nubmers
#ifndef ttreal
#define ttreal float
#define TT_ONE 1.0f
#define TT_ZERO 0.0f
#define TT_TWO 2.0f
#define TT_HALF 0.5f
#define TT_360 360.0f
#define TT_DEG2RAD_F 0.0174532925f
#define TT_RAD2DEG_F 57.295779513f
#define TT_PI32 3.14159265359f
#define tt_sqrt(x) sqrtf(x)
#define tt_sin(x) sinf(x)
#define tt_cos(x) cosf(x)
#define tt_asin(x) asinf(x)
#define tt_atan2(x,y) atan2f(x,y)
#define tt_tan(x) tanf(x)
#define tt_abs(x) fabs(x)
#else
#if !defined(TT_ONE) || !defined(TT_ZERO) || !defined(TT_TWO) || !defined(TT_HALF) || !defined(TT_DEG2RAD_F) || !defined(TT_RAD2DEG_F) || !defined(TT_360) || !defined(TT_PI32)
#warning "ttreal redefined but not all constants are, trying to use float"
#define TT_ONE 1.0f
#define TT_ZERO 0.0f
#define TT_TWO 2.0f
#define TT_HALF 0.5f
#define TT_360 360.0f
#define TT_DEG2RAD_F 0.0174532925f
#define TT_RAD2DEG_F 57.295779513f
#define TT_PI32 3.14159265359f
#endif
#ifndef tt_sqrt
#error "if you redefine ttreal, you must define tt_sqrt (libc's sqrtf by default)"
#endif
#ifndef tt_sin
#error "if you redefine ttreal, you must define tt_sin (libc's sinf by default)"
#endif
#ifndef tt_cos
#error "if you redefine ttreal, you must define tt_cos (libc's cosf by default)"
#endif
#ifndef tt_tan
#error "if you redefine ttreal, you must define tt_tan (libc's tanf by default)"
#endif 
#ifndef tt_asin
#error "if you redefine ttreal, you must define tt_asin (libc's asinf by default)"
#endif
#ifndef tt_atan2
#error "if you redefine ttreal, you must define tt_atan2 (libc's atan2f by default)"
#endif
#ifndef tt_abs
#error "if you redefine ttreal, you must define tt_abs (libc's fabs by default)"
#endif

#endif

typedef struct V2
{
    ttreal x,y;
} V2;

typedef struct V3
{
    ttreal x, y, z;
} V3;

typedef struct V4
{
    ttreal x, y, z, w;
} V4;

typedef struct Ray
{
    V3 origin;
    V3 dir;
} Ray;

// quaternion for rotations
typedef struct Quat
{
    ttreal w, x, y, z;
} Quat;

// column major layout
typedef struct Mat4
{
    union {
        struct
        {
            ttreal m[16];
        } ;

        struct
        {
            ttreal cr[4][4]; // column row
        };

        struct
        {
            ttreal m11, m21, m31, m41;
            ttreal m12, m22, m32, m42;
            ttreal m13, m23, m33, m43;
            ttreal m14, m24, m34, m44;
        };
    };
}Mat4;

// column major layout
typedef struct Mat3
{
    union {
        struct
        {
            ttreal m[9];
        };
        struct 
        {
            ttreal cr[3][3]; // column row
        };
        struct {
            ttreal m11, m21, m31;
            ttreal m12, m22, m32;
            ttreal m13, m23, m33;
        };
    };
} Mat3;

extern Quat QUAT_IDENTITY;
extern V3 V3_ZERO;
extern V3 V3_ONE;

#ifdef AIKE_X86

typedef struct Mat4_sse2
{
    union {
        struct {
            __m128 m11, m21, m31, m41;
            __m128 m12, m22, m32, m42;
            __m128 m13, m23, m33, m43;
            __m128 m14, m24, m34, m44;
        } ;
        struct
        {
            __m128 mm[4][4];
        };
        struct
        {
            __m128 m[16];
        };
        struct
        {
            float f[16][4];
        };
    };
}Mat4_sse2;


static inline void mat4_mul_sse2(struct Mat4_sse2 *m, struct Mat4_sse2 *l, struct Mat4_sse2 *r)
{
    for (int row = 0; row < 4; row++)
	{
		for (int col = 0; col < 4; col++)
		{
            m->mm[row][col] = _mm_add_ps(_mm_mul_ps(l->mm[0][col], r->mm[row][0]), 
                    _mm_add_ps(_mm_mul_ps(l->mm[1][col], r->mm[row][1]), 
                    _mm_add_ps(_mm_mul_ps(l->mm[2][col], r->mm[row][2]), 
                    _mm_mul_ps(l->mm[3][col], r->mm[row][3]))));
		}
	}
}

static inline void mat4_load_sse2(struct Mat4_sse2 *dst, struct Mat4* m1, struct Mat4 *m2, struct Mat4 *m3, struct Mat4 *m4)
{
    for(int i = 0; i < 16; i++)
    {
        _Alignas(16) float data[4] = {m1->m[i], m2->m[i], m3->m[i], m4->m[i]};
        dst->m[i] = _mm_load_ps(data);
    }
}

static inline void mat4_extract_sse2(struct Mat4 *dst, struct Mat4_sse2 *src, u32 idx)
{
    _Alignas(16) float data[4];
    for(int i = 0; i < 16; i++)
    {
        _mm_store_ps(data, src->m[i]);
        dst->m[i] = data[idx];
    }
}

static inline void mat4_extract_all_sse2(struct Mat4 *restrict dst, struct Mat4_sse2 *restrict src)
{
    _Alignas(16) float data[4];
    for(int i = 0; i < 16; i++)
    {
        _mm_store_ps(data, src->m[i]);
        dst[0].m[i] = data[0];
        dst[1].m[i] = data[1];
        dst[2].m[i] = data[2];
        dst[3].m[i] = data[3];
    }
}

#else

typedef struct Mat4_sse2
{
    union {
        struct {
            struct Mat4 mat[4];
        }mats;
        struct
        {
            float f[16][4];
        };
    };
} Mat4_sse2;

static inline void mat4_mul_sse2(struct Mat4_sse2 *m, struct Mat4_sse2 *l, struct Mat4_sse2 *r)
{
    assert(false);
}

static inline void mat4_load_sse2(struct Mat4_sse2 *dst, struct Mat4* m1, struct Mat4 *m2, struct Mat4 *m3, struct Mat4 *m4)
{
    assert(false);
}

static inline void mat4_extract_sse2(struct Mat4 *dst, struct Mat4_sse2 *src, unsigned int idx)
{
    assert(false);
}

static inline void mat4_extract_all_sse2(struct Mat4 *restrict dst, struct Mat4_sse2 *restrict src)
{
    assert(false);
}


#endif

static inline struct V2 make_v2(ttreal x, ttreal y)
{
    struct V2 ret;
    ret.x = x;
    ret.y = y;
    return ret;
}

static inline struct V3 make_v3(ttreal x, ttreal y, ttreal z)
{
    struct V3 ret = {x, y, z};
    return ret;
}

static inline struct V4 make_v4(ttreal x, ttreal y, ttreal z, ttreal w)
{
    struct V4 ret = {x, y, z, w};
    return ret;
}

static inline struct Quat make_quat(ttreal w, ttreal x, ttreal y, ttreal z)
{
    struct Quat ret = {w, x, y, z};
    return ret;
}

static inline V2 v2_add(V2 *restrict res, V2 lhs, V2 rhs)
{
    res->x = lhs.x + rhs.x;
    res->y = lhs.y + rhs.y;
    return *res;
}

static inline V2 v2_sub(V2 *restrict res, V2 lhs, V2 rhs)
{
    res->x = lhs.x - rhs.x;
    res->y = lhs.y - rhs.y;
    return *res;
}

static inline void v2_normalize(V2 *v)
{
    ttreal len = tt_sqrt(v->x*v->x + v->y*v->y);
    v->x = v->x / len;
    v->y = v->y / len;
}

static inline ttreal v2_dot(V2 l, V2 r)
{
    return l.x*r.x + l.y*r.y;
}

static inline ttreal v2_len(V2 v)
{
    return tt_sqrt(v.x*v.x + v.y*v.y);
}

static inline V2 v2_scale(V2 v, float scale) {
    return (V2) {v.x*scale, v.y*scale};
}

static inline V2 v2_rotate(V2 v, float angleRad) {
    return (V2){cosf(angleRad)*v.x - sinf(angleRad)*v.y, sinf(angleRad)*v.x + cosf(angleRad)*v.y};
}

static inline V3 v3_add(V3 *restrict res, V3 lhs, V3 rhs)
{
    res->x = lhs.x + rhs.x;
    res->y = lhs.y + rhs.y;
    res->z = lhs.z + rhs.z;
    return *res;
}

static inline V3 v3_scale(V3 v, ttreal scale) {
    return make_v3(v.x * scale, v.y * scale, v.z * scale);
}

// just a division
static inline V3 v3_inv_scale(V3 v, ttreal scale) {
    return make_v3(v.x/scale, v.y/scale, v.z/scale);
}

static inline V3 v3_sub(V3 *restrict res, V3 lhs, V3 rhs)
{
    res->x = lhs.x - rhs.x;
    res->y = lhs.y - rhs.y;
    res->z = lhs.z - rhs.z;
    return *res;
}

static inline void v3_normalize(V3 *v)
{
    ttreal len = tt_sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
    v->x /= len;
    v->y /= len;
    v->z /= len;
}

static inline float v3_dot(V3 l, V3 r) {
    return l.x*r.x + l.y*r.y + l.z*r.z;
}

static inline float v3_len(V3 v) {
    return tt_sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static inline bool v3_hasnan(V3 v)
{
    return isnan(v.x) || isnan(v.y) || isnan(v.z);
}

static inline V3 v3_lerp(V3 a, V3 b, ttreal t)
{
    return (V3){a.x + t*(b.x-a.x), a.y + t*(b.y-a.y), a.z + t*(b.z-a.z)};
}

static inline Quat quat_identity()
{
    return (Quat){ .w = TT_ONE, .x = TT_ZERO, .y = TT_ZERO, .z = TT_ZERO};
}

static inline void quat_normalize(Quat *q) {
    ttreal invLen = TT_ONE/tt_sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
    q->w *= invLen;
    q->x *= invLen;
    q->y *= invLen;
    q->z *= invLen;
}

// normalized linear interpolation (not constant angular speed!)
static inline Quat quat_nlerp(Quat a, Quat b, ttreal t) {
    Quat ret = (Quat){a.w + t*(b.w-a.w), a.x + t*(b.x-a.x), a.y + t*(b.y-a.y), a.z + t*(b.z-a.z)};
    quat_normalize(&ret);
    return ret;
}

static inline void mat4_v4_mul(V4 *restrict res, struct Mat4 *restrict m, V4 v)
{
    res->x = v.x * m->m11 + v.y * m->m12 + v.z * m->m13 + v.w * m->m14;
    res->y = v.x * m->m21 + v.y * m->m22 + v.z * m->m23 + v.w * m->m24;
    res->z = v.x * m->m31 + v.y * m->m32 + v.z * m->m33 + v.w * m->m34;
    res->w = v.x * m->m41 + v.y * m->m42 + v.z * m->m43 + v.w * m->m44;
}

static inline void quat_angle_axis(struct Quat *q, ttreal angleDeg, struct V3 axis)
{
    ttreal halfAngleRad = (TT_DEG2RAD_F*angleDeg) / TT_TWO;
    ttreal sinHAR = tt_sin(halfAngleRad);
    q->w = tt_cos(halfAngleRad);
    v3_normalize(&axis);
    q->x = axis.x * sinHAR;
    q->y = axis.y * sinHAR;
    q->z = axis.z * sinHAR;
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
static inline void euler_from_quat(struct V3 *v, struct Quat q) 
{
    ttreal sinr_cosp = TT_TWO*(q.w*q.x + q.z*q.y);
    ttreal cosr_cosp = TT_ONE - TT_TWO*(q.x*q.x + q.z*q.z);
    v->z = tt_atan2(sinr_cosp, cosr_cosp);

    ttreal sinp = TT_TWO*(q.w*q.z - q.z*q.x);
    if(tt_abs(sinp) >= TT_ONE) {
        v->x = sinp < TT_ZERO ? -TT_PI32/TT_TWO : TT_PI32/TT_TWO;
    } else {
        v->x = tt_asin(sinp);
    }
    ttreal siny_cosp = TT_TWO*(q.w*q.y + q.x*q.z);
    ttreal cosy_cosp = TT_ONE - TT_TWO*(q.z*q.z + q.y*q.y);
    v->y = tt_atan2(siny_cosp, cosy_cosp);
}

static inline void euler_from_quat_deg(struct V3 *v, struct Quat q) 
{
    euler_from_quat(v, q);
    v->x *= TT_RAD2DEG_F;
    v->y *= TT_RAD2DEG_F;
    v->z *= TT_RAD2DEG_F;
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
static inline void quat_euler(struct Quat *q, struct V3 euler)
{
	ttreal cy = tt_cos(euler.y * TT_HALF);
	ttreal sy = tt_sin(euler.y * TT_HALF);
	ttreal cp = tt_cos(euler.x * TT_HALF);
	ttreal sp = tt_sin(euler.x * TT_HALF);
	ttreal cr = tt_cos(euler.z * TT_HALF);
	ttreal sr = tt_sin(euler.z * TT_HALF);
	q->w = cy * cp * cr + sy * sp * sr;
	q->x = sy * cp * sr + cy * sp * cr;
	q->y = sy * cp * cr - cy * sp * sr;
	q->z = cy * cp * sr - sy * sp * cr;
}

// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
static inline void quat_euler_deg(struct Quat *restrict q, struct V3 euler)
{
    euler.x *= TT_DEG2RAD_F;
    euler.y *= TT_DEG2RAD_F;
    euler.z *= TT_DEG2RAD_F;
    quat_euler(q, euler);
}

static inline void quat_mul(Quat *restrict res, Quat l, Quat r)
{
	res->w = l.w * r.w - l.x * r.x - l.y * r.y - l.z * r.z;
	res->x = l.w * r.x + l.x * r.w + l.y * r.z - l.z * r.y;
	res->y = l.w * r.y - l.x * r.z + l.y * r.w + l.z * r.x;
	res->z = l.w * r.z + l.x * r.y - l.y * r.x + l.z * r.w;
}

static inline struct V3 normalize_degrees(struct V3 degRot)
{
    degRot.x = degRot.x < TT_ZERO ? TT_360 + degRot.x : degRot.x;
    degRot.y = degRot.y < TT_ZERO ? TT_360 + degRot.y : degRot.y;
    degRot.z = degRot.z < TT_ZERO ? TT_360 + degRot.z : degRot.z;
    return (struct V3) {fmodf(degRot.x, TT_360), fmodf(degRot.y, TT_360), fmodf(degRot.z, TT_360)};
}

// fov is vertical (i think)
static inline void mat4_perspective(struct Mat4 *m, ttreal fov, ttreal aspect, ttreal zNear, ttreal zFar)
{
    const ttreal tanHalfFOV = tt_tan(fov*TT_DEG2RAD_F*TT_HALF);
    ttreal depth = zNear - zFar;
    m->m11 = TT_ONE / (tanHalfFOV * aspect);
    m->m21 = TT_ZERO;
    m->m31 = TT_ZERO;
    m->m41 = TT_ZERO;

    m->m12 = TT_ZERO;
    m->m22 = TT_ONE / tanHalfFOV;
    m->m32 = TT_ZERO;
    m->m42 = TT_ZERO;

    m->m13 = TT_ZERO;
    m->m23 = TT_ZERO;
    m->m33 = (-zNear - zFar) / depth;
    m->m43 = TT_ONE;

    m->m14 = TT_ZERO;
    m->m24 = TT_ZERO;
    m->m34 = TT_TWO * zFar * zNear / depth;
    m->m44 = TT_ZERO;
}

static inline void inverse_perspective(Mat4 *dest, Mat4 *perspectiveMat)
{
    ttreal a = perspectiveMat->m[0];
    ttreal b = perspectiveMat->m[5];
    ttreal c = perspectiveMat->m[10];
    ttreal d = perspectiveMat->m[14];
    ttreal e = perspectiveMat->m[11];

    for(int i = 0; i < 16; i++)
        dest->m[i]  = TT_ZERO;

    dest->m[0]  = TT_ONE / a;
    dest->m[5]  = TT_ONE / b;
    dest->m[11] = TT_ONE / d;
    dest->m[14] = TT_ONE / e;
    dest->m[15] = -c / (d * e);
}

static inline void mat4_ortho(struct Mat4 *m, ttreal l, ttreal r, ttreal b, ttreal t, ttreal zn, ttreal zf)
{
    m->m11 = TT_TWO / (r - l);
    m->m21 = TT_ZERO;
    m->m31 = TT_ZERO;
    m->m41 = TT_ZERO;

    m->m12 = TT_ZERO;
    m->m22 = TT_TWO / (t - b);
    m->m32 = TT_ZERO;
    m->m42 = TT_ZERO;

    m->m13 = TT_ZERO;
    m->m23 = TT_ZERO;
    m->m33 = -TT_TWO/(zf-zn);
    m->m43 = TT_ZERO;

    m->m14 = -(r+l)/(r-l);
    m->m24 = -(t+b)/(t-b);
    m->m34 = -(zf+zn)/(zf-zn);
    m->m44 = TT_ONE;
}

static inline void mat4_rotation(struct Mat4 *restrict m, struct Quat *restrict q)
{
    m->m11 = TT_ONE - TT_TWO*q->y*q->y - TT_TWO*q->z*q->z;
	m->m21 = TT_TWO*q->x*q->y + TT_TWO*q->z*q->w;
	m->m31 = TT_TWO*q->x*q->z - TT_TWO*q->y*q->w;
	m->m41 = TT_ZERO;

	m->m12 = TT_TWO*q->x*q->y - TT_TWO*q->z*q->w;
	m->m22 = TT_ONE - TT_TWO*q->x*q->x - TT_TWO*q->z*q->z;
	m->m32 = TT_TWO*q->y*q->z + TT_TWO*q->x*q->w;
	m->m42 = TT_ZERO;

	m->m13 = TT_TWO*q->x*q->z + TT_TWO*q->y*q->w;
	m->m23 = TT_TWO*q->y*q->z - TT_TWO*q->x*q->w;
	m->m33 = TT_ONE - TT_TWO*q->x*q->x - TT_TWO*q->y*q->y;
	m->m43 = TT_ZERO;

	m->m14 = TT_ZERO;
	m->m24 = TT_ZERO;
	m->m34 = TT_ZERO;
	m->m44 = TT_ONE;
}

static inline void mat3_rotation(struct Mat3 *restrict m, struct Quat *restrict q) {
    m->m11 = TT_ONE - TT_TWO*q->y*q->y - TT_TWO*q->z*q->z;
	m->m21 = TT_TWO*q->x*q->y + TT_TWO*q->z*q->w;
	m->m31 = TT_TWO*q->x*q->z - TT_TWO*q->y*q->w;

	m->m12 = TT_TWO*q->x*q->y - TT_TWO*q->z*q->w;
	m->m22 = TT_ONE - TT_TWO*q->x*q->x - TT_TWO*q->z*q->z;
	m->m32 = TT_TWO*q->y*q->z + TT_TWO*q->x*q->w;

	m->m13 = TT_TWO*q->x*q->z + TT_TWO*q->y*q->w;
	m->m23 = TT_TWO*q->y*q->z - TT_TWO*q->x*q->w;
	m->m33 = TT_ONE - TT_TWO*q->x*q->x - TT_TWO*q->y*q->y;
}

static inline void mat4_identity(struct Mat4 *m)
{
    m->m11 = TT_ONE; m->m21 = TT_ZERO; m->m31 = TT_ZERO; m->m41 = TT_ZERO;
    m->m12 = TT_ZERO; m->m22 = TT_ONE; m->m32 = TT_ZERO; m->m42 = TT_ZERO;
    m->m13 = TT_ZERO; m->m23 = TT_ZERO; m->m33 = TT_ONE; m->m43 = TT_ZERO;
    m->m14 = TT_ZERO; m->m24 = TT_ZERO; m->m34 = TT_ZERO; m->m44 = TT_ONE;
}

// opencv camera matrix (normally 3x4 form)
static inline void mat4_cv_camera(struct Mat4 *restrict m, struct Mat3 *restrict intrinsic, struct Mat4 *restrict extrinsic) {
    m->m11 = intrinsic->m13*extrinsic->m31 + intrinsic->m11*extrinsic->m11; 
    m->m12 = intrinsic->m13*extrinsic->m32 + intrinsic->m11*extrinsic->m12; 
    m->m13 = intrinsic->m13*extrinsic->m33 + intrinsic->m11*extrinsic->m13; 
    m->m14 = intrinsic->m13*extrinsic->m34 + intrinsic->m11*extrinsic->m14;

    m->m21 = intrinsic->m23*extrinsic->m31 + intrinsic->m22*extrinsic->m21; 
    m->m22 = intrinsic->m23*extrinsic->m32 + intrinsic->m22*extrinsic->m22; 
    m->m23 = intrinsic->m23*extrinsic->m33 + intrinsic->m22*extrinsic->m23; 
    m->m24 = intrinsic->m23*extrinsic->m34 + intrinsic->m22*extrinsic->m24;

    m->m31 = extrinsic->m31; 
    m->m32 = extrinsic->m32; 
    m->m33 = extrinsic->m33; 
    m->m34 = extrinsic->m34;

    m->m41 = 0.0f; m->m42 = 0.0f; m->m43 = 0.0f; m->m44 = 1.0f;
}

static inline void mat4_mul(struct Mat4 *restrict m, struct Mat4 *restrict l, struct Mat4 *restrict r)
{
/*	
    memset(m, 0, sizeof(struct Mat4));
    for (int row = 0; row < 4; row++)
	{
		for (int col = 0; col < 4; col++)
		{
			for (int n = 0; n < 4; n++)
			{
				m->cr[row][col] += l->cr[n][col] * r->cr[row][n];
			}
		}
	}
*/
    // compiler just fails to unroll it, this is 2x faster
    m->m11 = l->m11 * r->m11 + l->m12 * r->m21 + l->m13 * r->m31 + l->m14 * r->m41;
    m->m21 = l->m21 * r->m11 + l->m22 * r->m21 + l->m23 * r->m31 + l->m24 * r->m41;
    m->m31 = l->m31 * r->m11 + l->m32 * r->m21 + l->m33 * r->m31 + l->m34 * r->m41;
    m->m41 = l->m41 * r->m11 + l->m42 * r->m21 + l->m43 * r->m31 + l->m44 * r->m41;
    m->m12 = l->m11 * r->m12 + l->m12 * r->m22 + l->m13 * r->m32 + l->m14 * r->m42;
    m->m22 = l->m21 * r->m12 + l->m22 * r->m22 + l->m23 * r->m32 + l->m24 * r->m42;
    m->m32 = l->m31 * r->m12 + l->m32 * r->m22 + l->m33 * r->m32 + l->m34 * r->m42;
    m->m42 = l->m41 * r->m12 + l->m42 * r->m22 + l->m43 * r->m32 + l->m44 * r->m42;
    m->m13 = l->m11 * r->m13 + l->m12 * r->m23 + l->m13 * r->m33 + l->m14 * r->m43;
    m->m23 = l->m21 * r->m13 + l->m22 * r->m23 + l->m23 * r->m33 + l->m24 * r->m43;
    m->m33 = l->m31 * r->m13 + l->m32 * r->m23 + l->m33 * r->m33 + l->m34 * r->m43;
    m->m43 = l->m41 * r->m13 + l->m42 * r->m23 + l->m43 * r->m33 + l->m44 * r->m43;
    m->m14 = l->m11 * r->m14 + l->m12 * r->m24 + l->m13 * r->m34 + l->m14 * r->m44;
    m->m24 = l->m21 * r->m14 + l->m22 * r->m24 + l->m23 * r->m34 + l->m24 * r->m44;
    m->m34 = l->m31 * r->m14 + l->m32 * r->m24 + l->m33 * r->m34 + l->m34 * r->m44;
    m->m44 = l->m41 * r->m14 + l->m42 * r->m24 + l->m43 * r->m34 + l->m44 * r->m44;
}

static inline void mat3_mul(struct Mat3 *restrict m, struct Mat3 *restrict l, struct Mat3 *restrict r)
{
    m->m11 = l->m11 * r->m11 + l->m12 * r->m21 + l->m13 * r->m31;
    m->m21 = l->m21 * r->m11 + l->m22 * r->m21 + l->m23 * r->m31;
    m->m31 = l->m31 * r->m11 + l->m32 * r->m21 + l->m33 * r->m31;
    m->m12 = l->m11 * r->m12 + l->m12 * r->m22 + l->m13 * r->m32;
    m->m22 = l->m21 * r->m12 + l->m22 * r->m22 + l->m23 * r->m32;
    m->m32 = l->m31 * r->m12 + l->m32 * r->m22 + l->m33 * r->m32;
    m->m13 = l->m11 * r->m13 + l->m12 * r->m23 + l->m13 * r->m33;
    m->m23 = l->m21 * r->m13 + l->m22 * r->m23 + l->m23 * r->m33;
    m->m33 = l->m31 * r->m13 + l->m32 * r->m23 + l->m33 * r->m33;
}


/*static void v4_mat3_mul(struct V4 *res, struct V4 *v, struct Mat4 *m)
{
    res->x = v->x*m->m11+v->y*m->m21+v->z*m->m31+v->w*m->m41;
    res->y = v->x*m->m12+v->y*m->m22+v->z*m->m32+v->w*m->m42;
    res->z = v->x*m->m13+v->y*m->m23+v->z*m->m33+v->w*m->m43;
    res->w = v->x*m->m14+v->y*m->m24+v->z*m->m34+v->w*m->m44;
}*/

// translate rotate scale
static inline void mat4_trs(struct Mat4 *res, struct V3 t, struct Quat r, struct V3 s)
{
    res->m11 = (TT_ONE-TT_TWO*(r.y*r.y+r.z*r.z))*s.x;
    res->m21 = (r.x*r.y+r.z*r.w)*s.x*TT_TWO;
    res->m31 = (r.x*r.z-r.y*r.w)*s.x*TT_TWO;
    res->m41 = TT_ZERO;
    res->m12 = (r.x*r.y-r.z*r.w)*s.y*TT_TWO;
    res->m22 = (TT_ONE-TT_TWO*(r.x*r.x+r.z*r.z))*s.y;
    res->m32 = (r.y*r.z+r.x*r.w)*s.y*TT_TWO;
    res->m42 = TT_ZERO;
    res->m13 = (r.x*r.z+r.y*r.w)*s.z*TT_TWO;
    res->m23 = (r.y*r.z-r.x*r.w)*s.z*TT_TWO;
    res->m33 = (TT_ONE-TT_TWO*(r.x*r.x+r.y*r.y))*s.z;
    res->m43 = TT_ZERO;
    res->m14 = t.x;
    res->m24 = t.y;
    res->m34 = t.z;
    res->m44 = TT_ONE;
}

// translate rotate
static inline void mat4_tr(struct Mat4 *res, struct V3 t, struct Quat r)
{
    res->m11 = (TT_ONE-TT_TWO*(r.y*r.y+r.z*r.z));
    res->m21 = (r.x*r.y+r.z*r.w)*TT_TWO;
    res->m31 = (r.x*r.z-r.y*r.w)*TT_TWO;
    res->m41 = TT_ZERO;
    res->m12 = (r.x*r.y-r.z*r.w)*TT_TWO;
    res->m22 = (TT_ONE-TT_TWO*(r.x*r.x+r.z*r.z));
    res->m32 = (r.y*r.z+r.x*r.w)*TT_TWO;
    res->m42 = TT_ZERO;
    res->m13 = (r.x*r.z+r.y*r.w)*TT_TWO;
    res->m23 = (r.y*r.z-r.x*r.w)*TT_TWO;
    res->m33 = (TT_ONE-TT_TWO*(r.x*r.x+r.y*r.y));
    res->m43 = TT_ZERO;
    res->m14 = t.x;
    res->m24 = t.y;
    res->m34 = t.z;
    res->m44 = TT_ONE;
}

// rotate translate
static inline void mat4_rt(struct Mat4 *res, struct Quat r, struct V3 t)
{
    Mat4 translate, rotate;
    mat4_identity(&translate);
    translate.m14 = t.x;
    translate.m24 = t.y;
    translate.m34 = t.z;
    mat4_rotation(&rotate, &r);
    mat4_mul(res, &rotate, &translate);
}

// translate scale
static inline void mat3_ts(Mat3 *restrict m, V2 translate, V2 scale)
{
   m->m11 = scale.x;
   m->m12 = TT_ZERO;
   m->m13 = scale.x*translate.x;
   m->m21 = TT_ZERO;
   m->m22 = scale.y;
   m->m23 = scale.y*translate.y;
   m->m31 = TT_ZERO;
   m->m32 = TT_ZERO;
   m->m33 = TT_ONE;
}

static inline void mat3_v3_mul(V3 *restrict res, Mat3 *restrict m, V3 v)
{
    res->x = v.x * m->m11 + v.y * m->m12 + v.z * m->m13;
    res->y = v.x * m->m21 + v.y * m->m22 + v.z * m->m23;
    res->z = v.x * m->m31 + v.y * m->m32 + v.z * m->m33;
}

static inline void quat_v3_mul_pos(V3 *restrict res, Quat q, V3 v)
{
    // TODO: optimize
    Mat4 mat;
    mat4_rotation(&mat, &q);
    V4 vh = make_v4(v.x, v.y, v.z, TT_ONE);
    V4 rh;
    mat4_v4_mul(&rh, &mat, vh);
    res->x = rh.x; res->y = rh.y; res->z = rh.z;
}

static inline void quat_v3_mul_dir(V3 *restrict res, Quat q, V3 v)
{
    // TODO: optimize
    Mat4 mat;
    mat4_rotation(&mat, &q);
    V4 vh = make_v4(v.x, v.y, v.z, TT_ZERO);
    V4 rh;
    mat4_v4_mul(&rh, &mat, vh);
    res->x = rh.x; res->y = rh.y; res->z = rh.z;
}

static inline bool ray_intersect_plane(V3 *res, Ray r, V4 p) {
    V3 n = (V3){p.x, p.y, p.z};
    float denom = v3_dot(r.dir,n);
    if(tt_abs(denom) <= FLT_EPSILON) {
        return 0;
    } 
    ttreal t = -(v3_dot(r.origin, n) + p.w) / denom;
    if(t < 0) {
        return 0;
    }
    *res = (V3) {t*r.dir.x, t*r.dir.y, t*r.dir.z};
    v3_add(res, r.origin, *res);
    return 1;
}

int lineCircleIntersection(V2 results[], V2 circlePos, float r, V2 point1, V2 point2);
int lineSegCircleIntersection(V2 results[], V2 circlePos, float r, V2 point1, V2 point2);
V2 closest_point_on_line_seg(V2 p, V2 start, V2 end);
float closest_point_on_line_seg_t(V2 p, V2 start, V2 end);


#ifdef __cplusplus
inline V2 operator+ (const V2& lhs, const V2& rhs) { return (V2){lhs.x+rhs.x, lhs.y+rhs.y };}
inline V2 operator- (const V2& lhs, const V2& rhs) { return (V2){lhs.x-rhs.x, lhs.y-rhs.y };}
inline V2 operator* (const ttreal lhs, const V2& rhs) { return (V2){lhs*rhs.x, lhs*rhs.y };}
inline V3 operator+ (const V3& lhs, const V3& rhs) { return (V3){lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z };}
inline V3 operator- (const V3& lhs, const V3& rhs) { return (V3){lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z };}
inline V3 operator* (const ttreal lhs, const V3& rhs) { return (V3){lhs*rhs.x, lhs*rhs.y, lhs*rhs.z };}
inline V3 operator/ (const V3& lhs, const ttreal rhs) { return (V3){lhs.x/rhs, lhs.y/rhs, lhs.z/rhs};}
inline V3 operator* (const Mat3& lhs, const V3& rhs) {
    V3 ret;
    mat3_v3_mul(&ret, (Mat3*)&lhs, rhs);
    return ret;
}
inline V4 operator* (const Mat4& lhs, const V4& rhs) {
    V4 ret;
    mat4_v4_mul(&ret, (Mat4*)&lhs, rhs);
    return ret;
}
inline Quat operator* (const Quat& lhs, const Quat& rhs) {
    Quat ret;
    quat_mul(&ret, lhs, rhs);
    return ret;
}
inline Mat3 operator* (const Mat3& lhs, const Mat3& rhs) {
    Mat3 ret;
    mat3_mul(&ret, (Mat3*)&lhs, (Mat3*)&rhs);
    return ret;
}
#endif

#endif /* !TTMATH_H */

#ifdef TTMATH_IMPLEMENTATION
Quat QUAT_IDENTITY = (Quat){.w = TT_ONE, .x = TT_ZERO, .y = TT_ZERO, .z = TT_ZERO};
V3 V3_ZERO = (V3){TT_ZERO, TT_ZERO, TT_ZERO};
V3 V3_ONE = (V3){TT_ZERO, TT_ZERO, TT_ZERO};

int lineCircleIntersection(V2 results[], V2 circlePos, float r, V2 point1, V2 point2) {
	V2 p1, p2;
	v2_sub(&p1, point1, circlePos);
	v2_sub(&p2, point2, circlePos);
	float m = (p2.y-p1.y)/(p2.x-p1.x);
	float b = (p1.y - m*p1.x);
    float det = (4.0f*m*m*b*b-4.0f*(1.0f+m*m)*(b*b-r*r));
	if(det >= 0.0f){
		float x1 = (-2.0f*m*b + sqrtf(det))/(2.0f*(1.0f+m*m));
		float x2 = (-2.0f*m*b - sqrtf(det))/(2.0f*(1.0f+m*m));
		float y1 = m*x1+b;
		float y2 = m*x2+b;
		results[0] = (V2){x1+circlePos.x, y1+circlePos.y};
		if(det > FLT_EPSILON) {
			results[1] = (V2){x2+circlePos.x, y2+circlePos.y};
			return 2;
		}
		return 1;
	}
	return 0;
}

int lineSegCircleIntersection(V2 results[], V2 circlePos, float r, V2 point1, V2 point2) {
    V2 lineInts[2];
    int count = lineCircleIntersection(lineInts, circlePos, r, point1, point2);
    int ret = 0;
    V2 dir;
    v2_sub(&dir, point2, point1);
    float len = v2_len(dir);
    v2_normalize(&dir);
    for(int i = 0; i < count; i++) {
        V2 toInt;
        v2_sub(&toInt, lineInts[i], point1);
        float dot = v2_dot(toInt, dir);
        if(dot >= 0.0f && dot <= len) {
            results[ret++] = lineInts[i];
        }
    }
    return ret;
}

V2 closest_point_on_line_seg(V2 p, V2 start, V2 end) {
    V2 dif, difp, ret;
    v2_sub(&dif, end, start);
    v2_sub(&difp, p, start);
    float len2 = dif.x*dif.x + dif.y*dif.y;
    float t = (difp.x*dif.x + difp.y*dif.y) / len2;
    t = TT_CLAMP(t, 0.0f, 1.0f);
    v2_add(&ret, start, (V2){dif.x*t, dif.y*t});
    return ret;
}

float closest_point_on_line_seg_t(V2 p, V2 start, V2 end) {
    V2 dif, difp, ret;
    v2_sub(&dif, end, start);
    v2_sub(&difp, p, start);
    float len2 = dif.x*dif.x + dif.y*dif.y;
    float t = (difp.x*dif.x + difp.y*dif.y) / len2;
    return t;
}

#endif
