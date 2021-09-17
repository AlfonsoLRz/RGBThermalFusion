#version 450

#extension GL_ARB_compute_variable_group_size : enable
layout(local_size_variable) in;

#define SVD_NUM_SWEEPS  10
#define SMALL_EPSILON   1.e-3

#include <Assets/Shaders/Compute/Templates/constraints.glsl>
#include <Assets/Shaders/Compute/Templates/modelStructs.glsl>

layout (std430, binding = 0) buffer PointBuffer      { vec4 points[]; };
layout (std430, binding = 1) buffer NeighborBuffer   { uint neighborIndex[]; };
layout (std430, binding = 2) buffer NormalBuffer     { vec4 normal[]; };

uniform uint arraySize;
uniform uint numNeighbors;
uniform uint offset;
uniform vec3 viewpoint;

void givens_coeffs_sym(float a_pp, float a_pq, float a_qq, out float c, out float s) {
    if (a_pq == 0.0f) 
    {
        c = 1.0f;
        s = 0.0f;
        return;
    }

    float tau = (a_qq - a_pp) / (2.0 * a_pq);
    float stt = sqrt(1.0 + tau * tau);
    float tan = 1.0 / ((tau >= 0.0) ? (tau + stt) : (tau - stt));

    c = inversesqrt(1.0 + tan * tan);   // 92
    s = tan * c;
}

void svd_rotate_xy(inout float x, inout float y, in float c, in float s) {
    float u = x; float v = y;
    x = c * u - s * v;
    y = s * u + c * v;
}

void svd_rotateq_xy(inout float x, inout float y, inout float a, in float c, in float s) {
    float cc = c * c; float ss = s * s;
    float mx = 2.0f * c * s * a;
    float u = x; float v = y;

    x = cc * u - mx + ss * v;
    y = ss * u + mx + cc * v;
}

void svd_rotate01(inout mat3 vtav, inout mat3 v) {
    if (vtav[0][1] == 0.0f) return;

    float c, s;
    givens_coeffs_sym(vtav[0][0], vtav[0][1], vtav[1][1], c, s);
    svd_rotateq_xy(vtav[0][0], vtav[1][1], vtav[0][1], c, s);
    svd_rotate_xy(vtav[0][2], vtav[1][2], c, s);
    vtav[0][1] = 0.0;

    svd_rotate_xy(v[0][0], v[0][1], c, s);
    svd_rotate_xy(v[1][0], v[1][1], c, s);
    svd_rotate_xy(v[2][0], v[2][1], c, s);
}

void svd_rotate02(inout mat3 vtav, inout mat3 v) {
    if (vtav[0][2] == 0.0f) return;

    float c, s;
    givens_coeffs_sym(vtav[0][0], vtav[0][2], vtav[2][2], c, s);
    svd_rotateq_xy(vtav[0][0], vtav[2][2], vtav[0][2], c, s);
    svd_rotate_xy(vtav[0][1], vtav[1][2], c, s);
    vtav[0][2] = 0.0;

    svd_rotate_xy(v[0][0], v[0][2], c, s);
    svd_rotate_xy(v[1][0], v[1][2], c, s);
    svd_rotate_xy(v[2][0], v[2][2], c, s);
}

void svd_rotate12(inout mat3 vtav, inout mat3 v) {
    if (vtav[1][2] == 0.0f) return;

    float c, s;
    givens_coeffs_sym(vtav[1][1], vtav[1][2], vtav[2][2], c, s);
    svd_rotateq_xy(vtav[1][1], vtav[2][2], vtav[1][2], c, s);
    svd_rotate_xy(vtav[0][1], vtav[0][2], c, s);
    vtav[1][2] = 0.0;

    svd_rotate_xy(v[0][1], v[0][2], c, s);
    svd_rotate_xy(v[1][1], v[1][2], c, s);
    svd_rotate_xy(v[2][1], v[2][2], c, s);
}

float svd_off(in mat3 a) {
    return sqrt(2.0f * ((a[0][1] * a[0][1]) + (a[0][2] * a[0][2]) + (a[1][2] * a[1][2])));
}

void svd_solve_sym(in mat3 a, out vec3 sigma, inout mat3 v) {
    // Assuming that A is symmetric: can optimize all operations for the lower left triagonal
    mat3 vtav = a;

    // Assuming V is identity: you can also pass a matrix the rotations should be applied to U is not computed
    for (int i = 0; i < SVD_NUM_SWEEPS; ++i) {
        if (svd_off(vtav) < SMALL_EPSILON)
            continue;

        svd_rotate01(vtav, v);
        svd_rotate02(vtav, v);
        svd_rotate12(vtav, v);
    }

    sigma = vec3(vtav[0][0], vtav[1][1], vtav[2][2]);
}

void qef_add(in vec3 p, in vec3 masspoint, inout mat3 ATA) {
    p -= masspoint;
    ATA[0][0] += p.x * p.x;
    ATA[0][1] += p.x * p.y;
    ATA[0][2] += p.x * p.z;
    ATA[1][1] += p.y * p.y;
    ATA[1][2] += p.y * p.z;
    ATA[2][2] += p.z * p.z;
}

void swap(inout float a, inout float b) {
    float x = a;
    a = b;
    b = x;
}

void swap_vec3(inout vec3 a, inout vec3 b) {
    vec3 x = a;
    a = b;
    b = -x;
}

mat3 transp(mat3 m) {
    return mat3(
        m[0][0], m[1][0], m[2][0],
        m[0][1], m[1][1], m[2][1],
        m[0][2], m[1][2], m[2][2]
    );
}

mat3 qef_solve(in mat3 ATA, out vec3 sigma) {
    mat3 V = mat3(1.0);

    svd_solve_sym(ATA, sigma, V);
    V = transp(V);

    if (sigma[0] < sigma[1]) 
    {
        swap(sigma[0], sigma[1]);
        swap_vec3(V[0], V[1]);
    }

    if (sigma[0] < sigma[2]) 
    {
        swap(sigma[0], sigma[2]);
        swap_vec3(V[0], V[2]);
    }

    if (sigma[1] < sigma[2]) 
    {
        swap(sigma[1], sigma[2]);
        swap_vec3(V[1], V[2]);
    }

    sigma = vec3(sqrt(sigma[0]), sqrt(sigma[1]), sqrt(sigma[2])) * 0.5f;

    return V;
}

void main()
{
    const uint index = gl_GlobalInvocationID.x;
    if (index >= arraySize) return;

    const uint baseIndex = index * numNeighbors;
    vec4 pointAccumulation = vec4(0.0f);
    uint numValidNeighbors = 0, validNeighbor;

    for (int i = 0; i < numNeighbors; ++i)
    {
        validNeighbor = uint(neighborIndex[baseIndex + i] != UINT_MAX);
        pointAccumulation += vec4(points[neighborIndex[baseIndex + i]].xyz, 1.0) * validNeighbor;
        numValidNeighbors += validNeighbor;
    }

    if (numValidNeighbors < 3)
    {
        normal[index] = vec4(.0f);
    }
    else
    {
        vec3 sigma;
        mat3 ATA = mat3(0.0f);
        const vec3 center = pointAccumulation.xyz / pointAccumulation.w;

        for (int i = 0; i < numValidNeighbors; ++i)
        {
            if (neighborIndex[baseIndex + i] != UINT_MAX)
            {
                qef_add(points[neighborIndex[baseIndex + i]].xyz, center, ATA);
            }
        }

        mat3 orientation = qef_solve(ATA, sigma);
        normal[index] = vec4(normalize((center + orientation[2] * sigma.y) - center), .0f);

        const vec3 viewpointVector = normalize(viewpoint);
        const float angle = acos(dot(normal[index].xyz, viewpointVector));

        if (angle >= PI / 2.0f)
        {
            normal[index] = -normal[index];
        }
    }
}