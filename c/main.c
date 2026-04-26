#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720
#define TRAIL_MAX 2400
#define GRID_EXTENT 9000
#define GRID_STEP 500
#define LAUNCH_POS V3(0.0f, 7.5f, 0.0f)
#define CHART_SAMPLES 180
#define MAX_AIRCRAFT 6
#define RADAR_X 110
#define RADAR_Y 540
#define RADAR_R 100

typedef enum SimPhase {
    PHASE_READY,
    PHASE_INTERCEPTING,
    PHASE_HIT,
    PHASE_MISSED
} SimPhase;

typedef struct Aircraft {
    Vector3 center;
    float radius;
    float altitude;
    float angle;
    float angularSpeed;
    Vector3 previousPosition;
    Vector3 position;
    Vector3 velocity;
    bool evading;
    Vector3 waypoints[8];
    int waypointCount;
    int waypointIndex;
    float maneuverPhase;
} Aircraft;

typedef struct Missile {
    Vector3 position;
    Vector3 velocity;
    Vector3 forward;
    Vector3 previousPosition;
    float speed;
    float age;
    float lateralAccel;
    float lateralDemand;
    float dragAccel;
    float thrustAccel;
    float trailTimer;
    bool active;
} Missile;

typedef struct Telemetry {
    float airDensity;
    float dynamicPressure;
    float closingSpeed;
    float range;
    float losRateDeg;
    float gLoad;
    float turnLimit;
} Telemetry;

typedef struct Charts {
    float speed[CHART_SAMPLES];
    float altitude[CHART_SAMPLES];
    float lateralG[CHART_SAMPLES];
    float accelerationG[CHART_SAMPLES];
    int index;
    int count;
    float timer;
} Charts;

typedef struct CameraRig {
    Vector3 offset;
    Vector3 target;
} CameraRig;

typedef struct ModelPose {
    Vector3 forward;
    float yawOffsetDeg;
} ModelPose;

typedef struct Trail {
    Vector3 points[TRAIL_MAX];
    int count;
} Trail;

static Vector3 V3(float x, float y, float z)
{
    return (Vector3){ x, y, z };
}

static float ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

static float RandomFloat(float minValue, float maxValue)
{
    return minValue + (maxValue - minValue) * ((float)GetRandomValue(0, 10000) / 10000.0f);
}

static float SmoothStep(float edge0, float edge1, float value)
{
    float t = ClampFloat((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static float AirDensity(float altitude)
{
    return 1.225f * expf(-ClampFloat(altitude, 0.0f, 18000.0f) / 8500.0f);
}

static Vector3 RotateVectorToward(Vector3 from, Vector3 to, float maxRadians)
{
    from = Vector3Normalize(from);
    to = Vector3Normalize(to);
    float dot = ClampFloat(Vector3DotProduct(from, to), -1.0f, 1.0f);
    float angle = acosf(dot);

    if (angle < 0.0001f || angle <= maxRadians) return to;

    float t = maxRadians / angle;
    Vector3 mixed = Vector3Add(Vector3Scale(from, sinf((1.0f - t) * angle)),
                               Vector3Scale(to, sinf(t * angle)));
    return Vector3Normalize(Vector3Scale(mixed, 1.0f / sinf(angle)));
}

static Vector3 RotateAroundAxis(Vector3 v, Vector3 axis, float angle)
{
    axis = Vector3Normalize(axis);
    float c = cosf(angle);
    float s = sinf(angle);
    return Vector3Add(Vector3Add(Vector3Scale(v, c),
                                 Vector3Scale(Vector3CrossProduct(axis, v), s)),
                      Vector3Scale(axis, Vector3DotProduct(axis, v) * (1.0f - c)));
}

static Vector3 BezierPoint(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
{
    float u = 1.0f - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;
    Vector3 p = Vector3Scale(p0, uuu);
    p = Vector3Add(p, Vector3Scale(p1, 3.0f * uu * t));
    p = Vector3Add(p, Vector3Scale(p2, 3.0f * u * tt));
    p = Vector3Add(p, Vector3Scale(p3, ttt));
    return p;
}

static float DistanceSegmentSegment(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2)
{
    Vector3 d1 = Vector3Subtract(q1, p1);
    Vector3 d2 = Vector3Subtract(q2, p2);
    Vector3 r = Vector3Subtract(p1, p2);
    float a = Vector3DotProduct(d1, d1);
    float e = Vector3DotProduct(d2, d2);
    float f = Vector3DotProduct(d2, r);
    float s = 0.0f;
    float t = 0.0f;

    if (a <= 0.0001f && e <= 0.0001f) return Vector3Distance(p1, p2);
    if (a <= 0.0001f) {
        t = ClampFloat(f / e, 0.0f, 1.0f);
    } else {
        float c = Vector3DotProduct(d1, r);
        if (e <= 0.0001f) {
            s = ClampFloat(-c / a, 0.0f, 1.0f);
        } else {
            float b = Vector3DotProduct(d1, d2);
            float denom = a * e - b * b;
            if (denom != 0.0f) s = ClampFloat((b * f - c * e) / denom, 0.0f, 1.0f);
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = ClampFloat(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = ClampFloat((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    Vector3 c1 = Vector3Add(p1, Vector3Scale(d1, s));
    Vector3 c2 = Vector3Add(p2, Vector3Scale(d2, t));
    return Vector3Distance(c1, c2);
}

static void DrawBezier3D(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, Color color)
{
    Vector3 prev = p0;
    for (int i = 1; i <= 48; i++) {
        float t = (float)i / 48.0f;
        Vector3 next = BezierPoint(p0, p1, p2, p3, t);
        DrawLine3D(prev, next, color);
        prev = next;
    }
}

static void AddTrailPoint(Trail *trail, Vector3 point)
{
    if (trail->count < TRAIL_MAX) {
        trail->points[trail->count++] = point;
        return;
    }

    for (int i = 1; i < TRAIL_MAX; i++) trail->points[i - 1] = trail->points[i];
    trail->points[TRAIL_MAX - 1] = point;
}

static void PushCharts(Charts *charts, Missile missile)
{
    charts->speed[charts->index] = missile.speed;
    charts->altitude[charts->index] = missile.position.y;
    charts->lateralG[charts->index] = missile.lateralAccel / 9.81f;
    charts->accelerationG[charts->index] = (missile.thrustAccel - missile.dragAccel) / 9.81f;
    charts->index = (charts->index + 1) % CHART_SAMPLES;
    if (charts->count < CHART_SAMPLES) charts->count++;
}

static void DrawTrail(Trail trail, Color color)
{
    for (int i = 1; i < trail.count; i++) {
        float t = (float)i / (float)trail.count;
        Color c = color;
        c.a = (unsigned char)(35.0f + 180.0f * t);
        DrawLine3D(trail.points[i - 1], trail.points[i], c);
    }
}

static Aircraft SpawnAircraft(void)
{
    Aircraft aircraft = { 0 };
    aircraft.center = V3(RandomFloat(-750.0f, 750.0f), 0.0f, RandomFloat(-750.0f, 750.0f));
    aircraft.radius = RandomFloat(2600.0f, 4300.0f);
    aircraft.altitude = RandomFloat(2200.0f, 5200.0f);
    aircraft.angle = RandomFloat(0.0f, PI * 2.0f);
    float fighterSpeed = RandomFloat(210.0f, 285.0f);
    aircraft.angularSpeed = fighterSpeed / aircraft.radius;
    if (GetRandomValue(0, 1) == 0) aircraft.angularSpeed *= -1.0f;
    aircraft.position = V3(aircraft.center.x + cosf(aircraft.angle) * aircraft.radius,
                           aircraft.altitude,
                           aircraft.center.z + sinf(aircraft.angle) * aircraft.radius);
    aircraft.velocity = V3(-sinf(aircraft.angle) * aircraft.radius * aircraft.angularSpeed,
                           0.0f,
                           cosf(aircraft.angle) * aircraft.radius * aircraft.angularSpeed);
    aircraft.previousPosition = aircraft.position;
    return aircraft;
}

static void GenerateEvadingWaypoints(Aircraft *a)
{
    a->waypointCount = 6 + GetRandomValue(0, 2);
    Vector3 current = a->position;
    for (int i = 0; i < a->waypointCount; i++) {
        current.x += RandomFloat(-1400.0f, 1400.0f);
        current.z += RandomFloat(-1400.0f, 1400.0f);
        float alt = a->altitude + RandomFloat(-900.0f, 900.0f);
        alt = ClampFloat(alt, 1600.0f, 5800.0f);
        a->waypoints[i] = V3(current.x, alt, current.z);
    }
}

static Aircraft SpawnEvadingAircraft(void)
{
    Aircraft aircraft = { 0 };
    aircraft.center = V3(RandomFloat(-750.0f, 750.0f), 0.0f, RandomFloat(-750.0f, 750.0f));
    aircraft.radius = RandomFloat(2600.0f, 4300.0f);
    aircraft.altitude = RandomFloat(2200.0f, 5200.0f);
    aircraft.angle = RandomFloat(0.0f, PI * 2.0f);
    float fighterSpeed = RandomFloat(230.0f, 280.0f);
    aircraft.angularSpeed = fighterSpeed / aircraft.radius;
    if (GetRandomValue(0, 1) == 0) aircraft.angularSpeed *= -1.0f;
    aircraft.position = V3(aircraft.center.x + cosf(aircraft.angle) * aircraft.radius,
                           aircraft.altitude,
                           aircraft.center.z + sinf(aircraft.angle) * aircraft.radius);
    aircraft.velocity = V3(-sinf(aircraft.angle) * aircraft.radius * aircraft.angularSpeed,
                           0.0f,
                           cosf(aircraft.angle) * aircraft.radius * aircraft.angularSpeed);
    aircraft.previousPosition = aircraft.position;
    aircraft.evading = true;
    aircraft.maneuverPhase = RandomFloat(0.0f, PI * 2.0f);
    GenerateEvadingWaypoints(&aircraft);
    aircraft.waypointIndex = 0;
    return aircraft;
}

static void UpdateAircraft(Aircraft *aircraft, float dt)
{
    aircraft->previousPosition = aircraft->position;

    if (aircraft->evading) {
        Vector3 target = aircraft->waypoints[aircraft->waypointIndex];
        Vector3 toTarget = Vector3Subtract(target, aircraft->position);
        float dist = Vector3Length(toTarget);

        if (dist < 350.0f) {
            aircraft->waypointIndex = (aircraft->waypointIndex + 1) % aircraft->waypointCount;
            target = aircraft->waypoints[aircraft->waypointIndex];
            toTarget = Vector3Subtract(target, aircraft->position);
        }

        Vector3 desiredDir = Vector3Normalize(toTarget);

        aircraft->maneuverPhase += dt * 1.8f;
        Vector3 up = V3(0.0f, 1.0f, 0.0f);
        float weave = sinf(aircraft->maneuverPhase) * 0.55f;
        desiredDir = RotateAroundAxis(desiredDir, up, weave);

        float climbDive = sinf(aircraft->maneuverPhase * 0.7f + 1.0f) * 0.35f;
        desiredDir.y += climbDive;
        desiredDir = Vector3Normalize(desiredDir);

        Vector3 currentDir = Vector3Normalize(aircraft->velocity);
        if (Vector3Length(currentDir) < 0.001f) currentDir = desiredDir;

        float maxTurn = 0.55f * dt;
        Vector3 newDir = RotateVectorToward(currentDir, desiredDir, maxTurn);

        float speed = Vector3Length(aircraft->velocity);
        speed = ClampFloat(speed + RandomFloat(-0.5f, 0.5f) * dt, 200.0f, 295.0f);

        aircraft->velocity = Vector3Scale(newDir, speed);
        aircraft->position = Vector3Add(aircraft->position, Vector3Scale(aircraft->velocity, dt));
        aircraft->altitude = aircraft->position.y;
    } else {
        aircraft->angle += aircraft->angularSpeed * dt;
        aircraft->position = V3(aircraft->center.x + cosf(aircraft->angle) * aircraft->radius,
                                aircraft->altitude,
                                aircraft->center.z + sinf(aircraft->angle) * aircraft->radius);
        aircraft->velocity = V3(-sinf(aircraft->angle) * aircraft->radius * aircraft->angularSpeed,
                                0.0f,
                                cosf(aircraft->angle) * aircraft->radius * aircraft->angularSpeed);
    }
}

static Vector3 PredictInterceptPoint(Vector3 shooter, Vector3 target, Vector3 targetVelocity, float missileSpeed, float *timeOut)
{
    Vector3 offset = Vector3Subtract(target, shooter);
    float a = Vector3DotProduct(targetVelocity, targetVelocity) - missileSpeed * missileSpeed;
    float b = 2.0f * Vector3DotProduct(offset, targetVelocity);
    float c = Vector3DotProduct(offset, offset);
    float t = 0.0f;

    if (fabsf(a) < 0.0001f) {
        t = (fabsf(b) > 0.0001f) ? -c / b : 0.0f;
    } else {
        float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f) {
            float root = sqrtf(disc);
            float t1 = (-b - root) / (2.0f * a);
            float t2 = (-b + root) / (2.0f * a);
            if (t1 > 0.0f && t2 > 0.0f) t = fminf(t1, t2);
            else t = fmaxf(t1, t2);
        }
    }

    if (t < 0.4f || !isfinite(t)) t = Vector3Length(offset) / fmaxf(missileSpeed, 1.0f);
    t = ClampFloat(t, 0.4f, 32.0f);
    if (timeOut) *timeOut = t;
    return Vector3Add(target, Vector3Scale(targetVelocity, t));
}

static int ParseObjFaceToken(const char *token, int *vertexIndex, int *texcoordIndex)
{
    int v = 0;
    int vt = 0;
    int parsed = sscanf(token, "%d/%d", &v, &vt);
    if (parsed < 1) return 0;

    *vertexIndex = v - 1;
    *texcoordIndex = (parsed >= 2) ? vt - 1 : -1;
    return 1;
}

static void CountObjMesh(const char *fileName, int *vertexCount, int *texcoordCount, int *triangleCount)
{
    FILE *file = fopen(fileName, "r");
    if (!file) return;

    char line[1024];
    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "v ", 2) == 0) {
            (*vertexCount)++;
        } else if (strncmp(line, "vt ", 3) == 0) {
            (*texcoordCount)++;
        } else if (strncmp(line, "f ", 2) == 0) {
            int faceVertices = 0;
            char *token = strtok(line + 2, " \t\r\n");
            while (token) {
                faceVertices++;
                token = strtok(NULL, " \t\r\n");
            }
            if (faceVertices >= 3) *triangleCount += faceVertices - 2;
        }
    }

    fclose(file);
}

static void PutMeshVertex(Mesh *mesh, int outIndex, Vector3 position, Vector3 normal, Vector2 texcoord)
{
    mesh->vertices[outIndex * 3 + 0] = position.x;
    mesh->vertices[outIndex * 3 + 1] = position.y;
    mesh->vertices[outIndex * 3 + 2] = position.z;

    mesh->normals[outIndex * 3 + 0] = normal.x;
    mesh->normals[outIndex * 3 + 1] = normal.y;
    mesh->normals[outIndex * 3 + 2] = normal.z;

    mesh->texcoords[outIndex * 2 + 0] = texcoord.x;
    mesh->texcoords[outIndex * 2 + 1] = texcoord.y;
}

static Model LoadMissileObjModel(const char *fileName, float scale)
{
    int sourceVertexCount = 0;
    int sourceTexcoordCount = 0;
    int triangleCount = 0;
    CountObjMesh(fileName, &sourceVertexCount, &sourceTexcoordCount, &triangleCount);

    if (sourceVertexCount == 0 || triangleCount == 0) {
        return LoadModelFromMesh(GenMeshCylinder(0.3f, 4.0f, 20));
    }

    Vector3 *positions = MemAlloc((unsigned int)sourceVertexCount * sizeof(Vector3));
    Vector2 *texcoords = MemAlloc((unsigned int)(sourceTexcoordCount > 0 ? sourceTexcoordCount : 1) * sizeof(Vector2));

    FILE *file = fopen(fileName, "r");
    if (!file) {
        MemFree(positions);
        MemFree(texcoords);
        return LoadModelFromMesh(GenMeshCylinder(0.3f, 4.0f, 20));
    }

    Vector3 minPoint = V3(0.0f, 0.0f, 0.0f);
    Vector3 maxPoint = V3(0.0f, 0.0f, 0.0f);
    int vertexAt = 0;
    int texcoordAt = 0;
    char line[1024];

    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "v ", 2) == 0) {
            Vector3 p = { 0 };
            if (sscanf(line, "v %f %f %f", &p.x, &p.y, &p.z) == 3) {
                positions[vertexAt++] = p;
                if (vertexAt == 1) {
                    minPoint = p;
                    maxPoint = p;
                } else {
                    minPoint = Vector3Min(minPoint, p);
                    maxPoint = Vector3Max(maxPoint, p);
                }
            }
        } else if (strncmp(line, "vt ", 3) == 0) {
            Vector2 uv = { 0 };
            if (sscanf(line, "vt %f %f", &uv.x, &uv.y) == 2) texcoords[texcoordAt++] = uv;
        }
    }

    Vector3 center = Vector3Scale(Vector3Add(minPoint, maxPoint), 0.5f);
    for (int i = 0; i < sourceVertexCount; i++) {
        positions[i] = Vector3Scale(Vector3Subtract(positions[i], center), scale);
    }

    Mesh mesh = { 0 };
    mesh.triangleCount = triangleCount;
    mesh.vertexCount = triangleCount * 3;
    mesh.vertices = MemAlloc((unsigned int)mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = MemAlloc((unsigned int)mesh.vertexCount * 3 * sizeof(float));
    mesh.texcoords = MemAlloc((unsigned int)mesh.vertexCount * 2 * sizeof(float));

    rewind(file);
    int outIndex = 0;
    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "f ", 2) != 0) continue;

        int faceVertexIndices[64] = { 0 };
        int faceTexcoordIndices[64] = { 0 };
        int faceCount = 0;

        char *token = strtok(line + 2, " \t\r\n");
        while (token && faceCount < 64) {
            if (ParseObjFaceToken(token, &faceVertexIndices[faceCount], &faceTexcoordIndices[faceCount])) faceCount++;
            token = strtok(NULL, " \t\r\n");
        }

        for (int i = 1; i < faceCount - 1; i++) {
            int vi[3] = { faceVertexIndices[0], faceVertexIndices[i], faceVertexIndices[i + 1] };
            int ti[3] = { faceTexcoordIndices[0], faceTexcoordIndices[i], faceTexcoordIndices[i + 1] };

            if (vi[0] < 0 || vi[1] < 0 || vi[2] < 0 ||
                vi[0] >= sourceVertexCount || vi[1] >= sourceVertexCount || vi[2] >= sourceVertexCount) {
                continue;
            }

            Vector3 p0 = positions[vi[0]];
            Vector3 p1 = positions[vi[1]];
            Vector3 p2 = positions[vi[2]];
            Vector3 normal = Vector3Normalize(Vector3CrossProduct(Vector3Subtract(p1, p0), Vector3Subtract(p2, p0)));
            if (Vector3Length(normal) < 0.001f) normal = V3(0.0f, 1.0f, 0.0f);

            for (int k = 0; k < 3; k++) {
                Vector2 uv = { 0.5f, 0.5f };
                if (ti[k] >= 0 && ti[k] < sourceTexcoordCount) uv = texcoords[ti[k]];
                PutMeshVertex(&mesh, outIndex++, positions[vi[k]], normal, uv);
            }
        }
    }

    fclose(file);
    MemFree(positions);
    MemFree(texcoords);

    if (outIndex != mesh.vertexCount) {
        mesh.vertexCount = outIndex;
        mesh.triangleCount = outIndex / 3;
    }

    UploadMesh(&mesh, false);
    return LoadModelFromMesh(mesh);
}

static Model LoadBinaryStlModel(const char *fileName, float targetLength)
{
    FILE *file = fopen(fileName, "rb");
    if (!file) return (Model){ 0 };

    unsigned char header[80];
    uint32_t triangleCount = 0;
    if (fread(header, 1, sizeof(header), file) != sizeof(header) ||
        fread(&triangleCount, sizeof(triangleCount), 1, file) != 1 ||
        triangleCount == 0) {
        fclose(file);
        return (Model){ 0 };
    }

    Mesh mesh = { 0 };
    mesh.triangleCount = (int)triangleCount;
    mesh.vertexCount = (int)triangleCount * 3;
    mesh.vertices = MemAlloc((unsigned int)mesh.vertexCount * 3 * sizeof(float));
    mesh.normals = MemAlloc((unsigned int)mesh.vertexCount * 3 * sizeof(float));

    Vector3 *raw = MemAlloc((unsigned int)mesh.vertexCount * sizeof(Vector3));
    Vector3 *normals = MemAlloc((unsigned int)mesh.vertexCount * sizeof(Vector3));
    Vector3 minPoint = V3(0.0f, 0.0f, 0.0f);
    Vector3 maxPoint = V3(0.0f, 0.0f, 0.0f);
    int vertexAt = 0;

    for (uint32_t tri = 0; tri < triangleCount; tri++) {
        float n[3] = { 0 };
        float v[9] = { 0 };
        unsigned short attribute = 0;
        if (fread(n, sizeof(float), 3, file) != 3 ||
            fread(v, sizeof(float), 9, file) != 9 ||
            fread(&attribute, sizeof(attribute), 1, file) != 1) {
            break;
        }

        Vector3 normal = Vector3Normalize(V3(n[0], n[1], n[2]));
        for (int k = 0; k < 3; k++) {
            Vector3 p = V3(v[k * 3 + 0], v[k * 3 + 2], v[k * 3 + 1]);
            raw[vertexAt] = p;
            normals[vertexAt] = Vector3Normalize(V3(normal.x, normal.z, normal.y));
            if (vertexAt == 0) {
                minPoint = p;
                maxPoint = p;
            } else {
                minPoint = Vector3Min(minPoint, p);
                maxPoint = Vector3Max(maxPoint, p);
            }
            vertexAt++;
        }
    }
    fclose(file);

    mesh.vertexCount = vertexAt;
    mesh.triangleCount = vertexAt / 3;
    Vector3 size = Vector3Subtract(maxPoint, minPoint);
    float maxDim = fmaxf(size.x, fmaxf(size.y, size.z));
    float scale = (maxDim > 0.001f) ? targetLength / maxDim : 1.0f;
    Vector3 center = Vector3Scale(Vector3Add(minPoint, maxPoint), 0.5f);

    for (int i = 0; i < mesh.vertexCount; i++) {
        Vector3 p = Vector3Scale(Vector3Subtract(raw[i], center), scale);
        mesh.vertices[i * 3 + 0] = p.x;
        mesh.vertices[i * 3 + 1] = p.y;
        mesh.vertices[i * 3 + 2] = p.z;
        mesh.normals[i * 3 + 0] = normals[i].x;
        mesh.normals[i * 3 + 1] = normals[i].y;
        mesh.normals[i * 3 + 2] = normals[i].z;
    }

    MemFree(raw);
    MemFree(normals);

    UploadMesh(&mesh, false);
    Model model = LoadModelFromMesh(mesh);
    if (model.materialCount > 0) model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = (Color){ 136, 144, 148, 255 };
    return model;
}

static void DrawModelFacing(Model model, Vector3 position, Vector3 direction, Color tint)
{
    Vector3 forward = Vector3Normalize(direction);
    if (Vector3Length(forward) < 0.001f) forward = V3(1.0f, 0.0f, 0.0f);

    Vector3 modelForward = V3(-1.0f, 0.0f, 0.0f);
    float dot = ClampFloat(Vector3DotProduct(modelForward, forward), -1.0f, 1.0f);
    float angle = acosf(dot) * RAD2DEG;
    Vector3 axis = Vector3CrossProduct(modelForward, forward);
    if (Vector3Length(axis) < 0.001f) axis = V3(0.0f, 1.0f, 0.0f);

    DrawModelEx(model, position, Vector3Normalize(axis), angle, V3(1.0f, 1.0f, 1.0f), tint);
}

static void DrawModelFacingAxis(Model model, Vector3 position, Vector3 direction, ModelPose pose, float scale, Color tint)
{
    Vector3 forward = Vector3Normalize(direction);
    if (Vector3Length(forward) < 0.001f) forward = V3(1.0f, 0.0f, 0.0f);
    Vector3 modelForward = Vector3Normalize(pose.forward);

    float dot = ClampFloat(Vector3DotProduct(modelForward, forward), -1.0f, 1.0f);
    float angle = acosf(dot) * RAD2DEG + pose.yawOffsetDeg;
    Vector3 axis = Vector3CrossProduct(modelForward, forward);
    if (Vector3Length(axis) < 0.001f) axis = V3(0.0f, 1.0f, 0.0f);

    DrawModelEx(model, position, Vector3Normalize(axis), angle, V3(scale, scale, scale), tint);
}

static void DrawAircraft(Aircraft aircraft, Model *planeModel)
{
    Vector3 dir = Vector3Normalize(aircraft.velocity);
    if (planeModel && planeModel->meshCount > 0) {
        DrawModelFacingAxis(*planeModel,
                            aircraft.position,
                            dir,
                            (ModelPose){ V3(0.0f, 0.0f, 1.0f), 180.0f },
                            1.0f,
                            (Color){ 136, 144, 148, 255 });
        return;
    }

    Vector3 up = V3(0.0f, 1.0f, 0.0f);
    Vector3 right = Vector3Normalize(Vector3CrossProduct(dir, up));
    Vector3 nose = Vector3Add(aircraft.position, Vector3Scale(dir, 3.8f));
    Vector3 tail = Vector3Subtract(aircraft.position, Vector3Scale(dir, 4.4f));

    DrawCylinderEx(tail, nose, 0.62f, 0.22f, 12, (Color){ 218, 222, 224, 255 });

    Vector3 wingCenter = Vector3Subtract(aircraft.position, Vector3Scale(dir, 0.4f));
    Vector3 wingL0 = Vector3Add(Vector3Subtract(wingCenter, Vector3Scale(right, 6.8f)), Vector3Scale(dir, 0.8f));
    Vector3 wingL1 = Vector3Add(Vector3Subtract(wingCenter, Vector3Scale(right, 6.8f)), Vector3Scale(dir, -1.1f));
    Vector3 wingR0 = Vector3Add(Vector3Add(wingCenter, Vector3Scale(right, 6.8f)), Vector3Scale(dir, 0.8f));
    Vector3 wingR1 = Vector3Add(Vector3Add(wingCenter, Vector3Scale(right, 6.8f)), Vector3Scale(dir, -1.1f));
    DrawTriangle3D(wingL0, wingR0, wingR1, (Color){ 190, 43, 48, 255 });
    DrawTriangle3D(wingL0, wingR1, wingL1, (Color){ 156, 32, 38, 255 });
    DrawTriangle3D(wingR1, wingR0, wingL0, (Color){ 190, 43, 48, 255 });
    DrawTriangle3D(wingL1, wingR1, wingL0, (Color){ 156, 32, 38, 255 });

    Vector3 tailCenter = Vector3Subtract(aircraft.position, Vector3Scale(dir, 3.7f));
    Vector3 tailL = Vector3Subtract(tailCenter, Vector3Scale(right, 2.4f));
    Vector3 tailR = Vector3Add(tailCenter, Vector3Scale(right, 2.4f));
    Vector3 tailBack = Vector3Subtract(tailCenter, Vector3Scale(dir, 1.2f));
    DrawTriangle3D(tailL, tailR, tailBack, (Color){ 56, 107, 204, 255 });
    DrawTriangle3D(tailBack, tailR, tailL, (Color){ 44, 84, 166, 255 });

    DrawTriangle3D(Vector3Add(tail, Vector3Scale(up, 0.25f)),
                   Vector3Add(tail, Vector3Scale(up, 2.2f)),
                   Vector3Add(tail, Vector3Scale(dir, 1.4f)),
                   (Color){ 56, 107, 204, 255 });
    DrawTriangle3D(Vector3Add(tail, Vector3Scale(dir, 1.4f)),
                   Vector3Add(tail, Vector3Scale(up, 2.2f)),
                   Vector3Add(tail, Vector3Scale(up, 0.25f)),
                   (Color){ 44, 84, 166, 255 });

    DrawSphere(nose, 0.33f, (Color){ 168, 207, 226, 255 });
}

static void DrawSkyAtmosphere(void)
{
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();

    for (int y = 0; y < screenHeight; y++) {
        float t = (float)y / (float)(screenHeight - 1);
        Color top = (Color){ 64, 142, 218, 255 };
        Color horizon = (Color){ 158, 205, 241, 255 };
        Color c = {
            (unsigned char)((1.0f - t) * top.r + t * horizon.r),
            (unsigned char)((1.0f - t) * top.g + t * horizon.g),
            (unsigned char)((1.0f - t) * top.b + t * horizon.b),
            255
        };
        DrawLine(0, y, screenWidth, y, c);
    }
}

static void DrawGroundGrid(void)
{
    for (int x = -GRID_EXTENT; x <= GRID_EXTENT; x += GRID_STEP) {
        Color c = (x == 0) ? (Color){ 88, 170, 220, 180 } : (Color){ 78, 88, 82, 118 };
        DrawLine3D(V3((float)x, 0.02f, -GRID_EXTENT), V3((float)x, 0.02f, GRID_EXTENT), c);
    }

    for (int z = -GRID_EXTENT; z <= GRID_EXTENT; z += GRID_STEP) {
        Color c = (z == 0) ? (Color){ 88, 170, 220, 180 } : (Color){ 78, 88, 82, 118 };
        DrawLine3D(V3(-GRID_EXTENT, 0.021f, (float)z), V3(GRID_EXTENT, 0.021f, (float)z), c);
    }
}

static void DrawPrediction(Missile missile, Aircraft aircraft, Vector3 interceptPoint, float interceptTime)
{
    Vector3 missileStart = missile.active ? missile.position : LAUNCH_POS;
    Vector3 missileDir = Vector3Normalize(Vector3Subtract(interceptPoint, missileStart));
    Vector3 aheadA = Vector3Add(missileStart, Vector3Scale(missileDir, 18.0f));
    Vector3 aheadB = Vector3Add(interceptPoint, V3(0.0f, 8.0f, 0.0f));
    DrawBezier3D(missileStart, aheadA, aheadB, interceptPoint, Fade(RED, 0.92f));

    Vector3 targetPast = Vector3Subtract(aircraft.position, Vector3Scale(aircraft.velocity, 2.4f));
    Vector3 targetFuture = Vector3Add(aircraft.position, Vector3Scale(aircraft.velocity, interceptTime));
    DrawBezier3D(targetPast,
                 Vector3Lerp(targetPast, aircraft.position, 0.55f),
                 Vector3Lerp(aircraft.position, targetFuture, 0.45f),
                 targetFuture,
                 Fade(SKYBLUE, 0.62f));

    DrawSphere(interceptPoint, 0.9f, Fade(YELLOW, 0.55f));
    DrawSphereWires(interceptPoint, 1.25f, 12, 12, Fade(YELLOW, 0.7f));
}

static void UpdateOrbitCamera(Camera3D *camera, CameraRig *rig, Missile missile, Aircraft aircraft, SimPhase phase)
{
    (void)aircraft;
    (void)phase;

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        rig->offset = RotateAroundAxis(rig->offset, V3(0.0f, 1.0f, 0.0f), -delta.x * 0.0052f);

        Vector3 viewDir = Vector3Normalize(Vector3Scale(rig->offset, -1.0f));
        Vector3 right = Vector3CrossProduct(viewDir, V3(0.0f, 1.0f, 0.0f));
        if (Vector3Length(right) < 0.001f) {
            right = Vector3CrossProduct(viewDir, V3(0.0f, 0.0f, 1.0f));
        }
        if (Vector3Length(right) < 0.001f) right = V3(1.0f, 0.0f, 0.0f);

        Vector3 nextOffset = RotateAroundAxis(rig->offset, right, -delta.y * 0.0038f);
        rig->offset = nextOffset;
    }

    float zoom = 1.0f - GetMouseWheelMove() * 0.12f;
    float dist = ClampFloat(Vector3Length(rig->offset) * zoom, 35.0f, 5200.0f);
    rig->offset = Vector3Scale(Vector3Normalize(rig->offset), dist);

    float vertical = fabsf(rig->offset.y) / fmaxf(Vector3Length(rig->offset), 1.0f);
    if (vertical > 0.9995f) {
        rig->offset.x += 0.001f * dist;
        rig->offset = Vector3Scale(Vector3Normalize(rig->offset), dist);
    }

    Vector3 focus = missile.active ? missile.position : LAUNCH_POS;
    rig->target = focus;
    camera->target = rig->target;
    camera->position = Vector3Add(camera->target, rig->offset);
}

static float ChartValue(const float values[CHART_SAMPLES], int count, int index, int offset)
{
    if (count <= 0) return 0.0f;
    int start = (index - count + CHART_SAMPLES) % CHART_SAMPLES;
    return values[(start + offset) % CHART_SAMPLES];
}

static void DrawChart(const char *label, const float values[CHART_SAMPLES], int count, int index,
                      Rectangle rect, float maxValue, Color color)
{
    DrawRectangleRec(rect, Fade(BLACK, 0.58f));
    DrawRectangleLinesEx(rect, 1.0f, Fade(RAYWHITE, 0.24f));
    DrawText(label, (int)rect.x + 8, (int)rect.y + 6, 14, Fade(RAYWHITE, 0.86f));

    if (count < 2 || maxValue <= 0.0f) return;

    Vector2 prev = { 0 };
    for (int i = 0; i < count; i++) {
        float v = ClampFloat(ChartValue(values, count, index, i) / maxValue, 0.0f, 1.0f);
        Vector2 p = {
            rect.x + 8.0f + ((float)i / (float)(count - 1)) * (rect.width - 16.0f),
            rect.y + rect.height - 8.0f - v * (rect.height - 28.0f)
        };
        if (i > 0) DrawLineV(prev, p, color);
        prev = p;
    }
}

static void DrawRadarScreen(Aircraft *aircrafts, int count, int selected, Vector3 launchPos)
{
    DrawCircle(RADAR_X, RADAR_Y, RADAR_R, Fade(BLACK, 0.72f));
    DrawCircleLines(RADAR_X, RADAR_Y, RADAR_R, Fade(GREEN, 0.85f));
    DrawCircleLines(RADAR_X, RADAR_Y, RADAR_R * 0.66f, Fade(GREEN, 0.4f));
    DrawCircleLines(RADAR_X, RADAR_Y, RADAR_R * 0.33f, Fade(GREEN, 0.4f));

    DrawLine(RADAR_X - RADAR_R, RADAR_Y, RADAR_X + RADAR_R, RADAR_Y, Fade(GREEN, 0.3f));
    DrawLine(RADAR_X, RADAR_Y - RADAR_R, RADAR_X, RADAR_Y + RADAR_R, Fade(GREEN, 0.3f));

    float maxRange = 6500.0f;
    for (int i = 0; i < count; i++) {
        float dx = aircrafts[i].position.x - launchPos.x;
        float dz = aircrafts[i].position.z - launchPos.z;
        float dist = sqrtf(dx * dx + dz * dz);
        if (dist > maxRange) continue;

        float rx = (dx / maxRange) * RADAR_R;
        float ry = (dz / maxRange) * RADAR_R;
        int bx = (int)(RADAR_X + rx);
        int by = (int)(RADAR_Y + ry);

        Color blipColor = (i == selected) ? RED : (aircrafts[i].evading ? ORANGE : GREEN);
        int radius = (i == selected) ? 5 : 3;
        DrawCircle(bx, by, radius, blipColor);
        if (i == selected) {
            DrawCircleLines(bx, by, 9, Fade(RED, 0.8f));
        }
    }

    DrawCircle(RADAR_X, RADAR_Y, 3, YELLOW);

    DrawText("RADAR", RADAR_X - 28, RADAR_Y - RADAR_R - 18, 16, Fade(GREEN, 0.9f));
    DrawText("TAB: cycle target", RADAR_X - 55, RADAR_Y + RADAR_R + 6, 14, Fade(RAYWHITE, 0.7f));
    DrawText("N", RADAR_X + RADAR_R + 8, RADAR_Y - 6, 12, GREEN);
    DrawText("E", RADAR_X + RADAR_R + 8, RADAR_Y + 8, 12, ORANGE);
}

static void DrawHud(SimPhase phase, bool paused, float timeScale, Aircraft aircraft, Missile missile, float interceptTime,
                    Telemetry telemetry, Charts charts)
{
    int screenWidth = GetScreenWidth();
    int screenHeight = GetScreenHeight();

    DrawRectangle(18, 18, 390, 218, Fade(BLACK, 0.62f));
    DrawRectangleLines(18, 18, 390, 218, Fade(RAYWHITE, 0.35f));

    char line[128];
    snprintf(line, sizeof(line), "Target speed:    %5.1f m/s", Vector3Length(aircraft.velocity));
    DrawText(line, 34, 34, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Target altitude: %5.1f m", aircraft.position.y);
    DrawText(line, 34, 60, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Lead solution:   %5.1f s", interceptTime);
    DrawText(line, 34, 86, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Missile speed:   %5.1f m/s", missile.active ? missile.speed : 0.0f);
    DrawText(line, 34, 112, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Air rho/q:       %.3f / %.0f", telemetry.airDensity, telemetry.dynamicPressure);
    DrawText(line, 34, 138, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Turn cmd/limit:  %.1fg / %.1fg", telemetry.gLoad, telemetry.turnLimit / 9.81f);
    DrawText(line, 34, 164, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Drag:            %.1f m/s2", missile.dragAccel);
    DrawText(line, 34, 190, 20, RAYWHITE);
    snprintf(line, sizeof(line), "Time scale:      %.2fx", timeScale);
    DrawText(line, 34, 216, 20, RAYWHITE);

    DrawChart("Speed", charts.speed, charts.count, charts.index,
              (Rectangle){ screenWidth - 278.0f, 36.0f, 250.0f, 72.0f }, 1700.0f, ORANGE);
    DrawChart("Altitude", charts.altitude, charts.count, charts.index,
              (Rectangle){ screenWidth - 278.0f, 116.0f, 250.0f, 72.0f }, 6500.0f, SKYBLUE);
    DrawChart("Lat g", charts.lateralG, charts.count, charts.index,
              (Rectangle){ screenWidth - 278.0f, 196.0f, 250.0f, 72.0f }, 55.0f, LIME);
    DrawChart("Accel g", charts.accelerationG, charts.count, charts.index,
              (Rectangle){ screenWidth - 278.0f, 276.0f, 250.0f, 72.0f }, 65.0f, YELLOW);

    const char *status = "I: launch interceptor";
    Color statusColor = RAYWHITE;
    if (paused) {
        status = "PAUSED - SPACE: resume";
        statusColor = YELLOW;
    } else if (phase == PHASE_INTERCEPTING) {
        status = "INTERCEPTOR GUIDING";
        statusColor = ORANGE;
    } else if (phase == PHASE_HIT) {
        status = "HIT - new target spawned";
        statusColor = GREEN;
    } else if (phase == PHASE_MISSED) {
        status = "MISS - press I for new shot";
        statusColor = RED;
    }
    DrawText(status, 18, screenHeight - 34, 20, statusColor);
    DrawText("I launch  SPACE pause  Arrow keys time  TAB target  Mouse orbit", screenWidth - 660, screenHeight - 34, 20, Fade(RAYWHITE, 0.78f));
}

int main(void)
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Ground-to-Air Missile Intercept Test");
    rlSetClipPlanes(0.5, 25000.0);
    SetTargetFPS(60);

    Model missileModel = LoadMissileObjModel("model/AVMT300.obj", 0.42f);
    Texture2D missileTexture = LoadTexture("model/Texture/texture.jpg");
    if (missileModel.materialCount > 0 && missileTexture.id > 0) {
        for (int i = 0; i < missileModel.materialCount; i++) {
            missileModel.materials[i].maps[MATERIAL_MAP_DIFFUSE].texture = missileTexture;
        }
    }

    Model planeModel = { 0 };
    bool hasPlaneModel = false;
    if (FileExists("model/uploads_files_2943574_MIG_airplane_lowpoly.stl")) {
        planeModel = LoadBinaryStlModel("model/uploads_files_2943574_MIG_airplane_lowpoly.stl", 42.0f);
        hasPlaneModel = planeModel.meshCount > 0;
    } else if (FileExists("model/plane.obj")) {
        planeModel = LoadMissileObjModel("model/plane.obj", 1.0f);
        hasPlaneModel = planeModel.meshCount > 0;
    }

    Camera3D camera = { 0 };
    camera.position = V3(28.0f, 18.0f, 32.0f);
    camera.target = V3(0.0f, 8.0f, 0.0f);
    camera.up = V3(0.0f, 1.0f, 0.0f);
    camera.fovy = 52.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    CameraRig cameraRig = {
        .offset = V3(90.0f, 45.0f, 135.0f),
        .target = LAUNCH_POS
    };

    Aircraft aircrafts[MAX_AIRCRAFT];
    for (int i = 0; i < MAX_AIRCRAFT / 2; i++) aircrafts[i] = SpawnAircraft();
    for (int i = MAX_AIRCRAFT / 2; i < MAX_AIRCRAFT; i++) aircrafts[i] = SpawnEvadingAircraft();
    int selectedAircraft = 0;
    Missile missile = {
        .position = LAUNCH_POS,
        .previousPosition = LAUNCH_POS,
        .forward = V3(0.0f, 1.0f, 0.0f)
    };
    Trail trail = { 0 };
    SimPhase phase = PHASE_READY;
    bool paused = false;
    float timeScale = 1.0f;
    float resultTimer = 0.0f;
    Vector3 explosionPosition = V3(0.0f, 0.0f, 0.0f);
    float interceptTime = 0.0f;
    Vector3 interceptPoint = V3(0.0f, 0.0f, 0.0f);
    Telemetry telemetry = { 0 };
    Charts charts = { 0 };
    double lastClickTime = -1.0;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();
        if (dt > 1.0f / 20.0f) dt = 1.0f / 20.0f;

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            double now = GetTime();
            if (lastClickTime > 0.0 && now - lastClickTime < 0.32) {
                ToggleBorderlessWindowed();
                lastClickTime = -1.0;
            } else {
                lastClickTime = now;
            }
        }

        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_LEFT)) timeScale = ClampFloat(timeScale * 0.5f, 0.05f, 4.0f);
        if (IsKeyPressed(KEY_RIGHT)) timeScale = ClampFloat(timeScale * 2.0f, 0.05f, 4.0f);
        if (IsKeyPressed(KEY_DOWN)) timeScale = 0.10f;
        if (IsKeyPressed(KEY_UP)) timeScale = 1.0f;
        if (IsKeyPressed(KEY_TAB)) selectedAircraft = (selectedAircraft + 1) % MAX_AIRCRAFT;

        float simDt = paused ? 0.0f : dt * timeScale;

        if ((phase == PHASE_READY || phase == PHASE_MISSED) && IsKeyPressed(KEY_I)) {
            paused = false;
            phase = PHASE_INTERCEPTING;
            missile.position = LAUNCH_POS;
            missile.previousPosition = missile.position;
            missile.forward = V3(0.0f, 1.0f, 0.0f);
            missile.speed = 145.0f;
            missile.velocity = Vector3Scale(missile.forward, missile.speed);
            missile.age = 0.0f;
            missile.lateralAccel = 0.0f;
            missile.lateralDemand = 0.0f;
            missile.dragAccel = 0.0f;
            missile.thrustAccel = 0.0f;
            missile.trailTimer = 0.0f;
            missile.active = true;
            trail.count = 0;
            charts = (Charts){ 0 };
        }

        if (!paused) {
            for (int i = 0; i < MAX_AIRCRAFT; i++) UpdateAircraft(&aircrafts[i], simDt);
        }

        interceptPoint = PredictInterceptPoint(missile.active ? missile.position : LAUNCH_POS,
                                               aircrafts[selectedAircraft].position,
                                               aircrafts[selectedAircraft].velocity,
                                               missile.active ? fmaxf(missile.speed, 1250.0f) : 1300.0f,
                                               &interceptTime);

        if (!paused && phase == PHASE_INTERCEPTING && missile.active) {
            missile.age += simDt;
            missile.previousPosition = missile.position;

            Vector3 toTarget = Vector3Subtract(aircrafts[selectedAircraft].position, missile.position);
            Vector3 lineOfSight = Vector3Normalize(toTarget);
            float closingSpeed = fmaxf(8.0f, -Vector3DotProduct(Vector3Subtract(aircrafts[selectedAircraft].velocity, missile.velocity), lineOfSight));
            float range = Vector3Length(toTarget);
            float dynamicLead = ClampFloat(range / closingSpeed, 0.10f, 30.0f);
            Vector3 leadPoint = Vector3Add(aircrafts[selectedAircraft].position, Vector3Scale(aircrafts[selectedAircraft].velocity, dynamicLead));
            Vector3 leadDirection = Vector3Normalize(Vector3Subtract(leadPoint, missile.position));
            Vector3 interceptDirection = Vector3Normalize(Vector3Subtract(interceptPoint, missile.position));
            float terminalBlend = 1.0f - SmoothStep(320.0f, 900.0f, range);

            float boostBlend = ClampFloat((missile.age - 0.32f) / 0.8f, 0.0f, 1.0f);
            Vector3 pnDirection = Vector3Normalize(Vector3Lerp(leadDirection, interceptDirection, 0.48f));
            Vector3 guidanceDirection = Vector3Normalize(Vector3Lerp(pnDirection, lineOfSight, terminalBlend * 0.72f));
            Vector3 desired = Vector3Normalize(Vector3Lerp(V3(0.0f, 1.0f, 0.0f), guidanceDirection, boostBlend));

            float rho = AirDensity(missile.position.y);
            float qbar = 0.5f * rho * missile.speed * missile.speed;
            float aeroAuthority = SmoothStep(18.0f, 72.0f, qbar);
            float boostAuthority = 1.0f - SmoothStep(1.7f, 3.1f, missile.age);
            float maxLateralAccel = (130.0f + 430.0f * aeroAuthority + 150.0f * boostAuthority) * boostBlend;
            float angleError = acosf(ClampFloat(Vector3DotProduct(missile.forward, desired), -1.0f, 1.0f));
            missile.lateralDemand = angleError * missile.speed / fmaxf(simDt, 0.001f);
            missile.lateralAccel = fminf(missile.lateralDemand, maxLateralAccel);
            float maxTurnRate = missile.lateralAccel / fmaxf(missile.speed, 8.0f);
            missile.forward = RotateVectorToward(missile.forward, desired, maxTurnRate * simDt);

            missile.thrustAccel = missile.age < 6.5f ? 560.0f * (1.0f - 0.56f * SmoothStep(4.4f, 6.5f, missile.age)) : 38.0f;
            missile.dragAccel = qbar * 0.00048f;
            float gravityAlongFlight = 9.81f * missile.forward.y;
            float speedDot = missile.thrustAccel - missile.dragAccel - gravityAlongFlight;
            missile.speed = ClampFloat(missile.speed + speedDot * simDt, 430.0f, 1900.0f);
            missile.velocity = Vector3Scale(missile.forward, missile.speed);
            missile.position = Vector3Add(missile.position, Vector3Scale(missile.velocity, simDt));
            missile.trailTimer += simDt;
            if (missile.trailTimer >= 0.016f) {
                missile.trailTimer = 0.0f;
                AddTrailPoint(&trail, missile.position);
            }
            charts.timer += dt;
            if (charts.timer >= 0.05f) {
                charts.timer = 0.0f;
                PushCharts(&charts, missile);
            }

            telemetry.airDensity = rho;
            telemetry.dynamicPressure = qbar;
            telemetry.closingSpeed = closingSpeed;
            telemetry.range = range;
            telemetry.gLoad = missile.lateralAccel / 9.81f;
            telemetry.turnLimit = maxLateralAccel;
            telemetry.losRateDeg = missile.lateralDemand / fmaxf(missile.speed, 1.0f) * RAD2DEG;

            float distanceToTarget = DistanceSegmentSegment(missile.previousPosition, missile.position,
                                                            aircrafts[selectedAircraft].previousPosition, aircrafts[selectedAircraft].position);
            if (distanceToTarget < 75.0f) {
                explosionPosition = aircrafts[selectedAircraft].position;
                bool wasEvading = aircrafts[selectedAircraft].evading;
                aircrafts[selectedAircraft] = wasEvading ? SpawnEvadingAircraft() : SpawnAircraft();
                phase = PHASE_HIT;
                missile.active = false;
                resultTimer = 0.0f;
                trail.count = 0;
                missile.position = LAUNCH_POS;
                missile.previousPosition = LAUNCH_POS;
                missile.forward = V3(0.0f, 1.0f, 0.0f);
                missile.velocity = V3(0.0f, 0.0f, 0.0f);
                missile.speed = 0.0f;
            } else if (missile.age > 42.0f || missile.position.y < 0.0f || Vector3Length(missile.position) > 11000.0f) {
                phase = PHASE_MISSED;
                missile.active = false;
                missile.position = LAUNCH_POS;
                missile.previousPosition = LAUNCH_POS;
                missile.forward = V3(0.0f, 1.0f, 0.0f);
                missile.velocity = V3(0.0f, 0.0f, 0.0f);
                missile.speed = 0.0f;
            }
        }

        if (!paused && phase == PHASE_HIT) {
            resultTimer += dt;
            if (resultTimer > 0.55f) {
                phase = PHASE_READY;
            }
        }

        if (!missile.active) {
            telemetry.airDensity = AirDensity(LAUNCH_POS.y);
            telemetry.dynamicPressure = 0.0f;
            telemetry.gLoad = 0.0f;
            telemetry.turnLimit = 0.0f;
        }

        UpdateOrbitCamera(&camera, &cameraRig, missile, aircrafts[selectedAircraft], phase);

        BeginDrawing();
        ClearBackground((Color){ 64, 142, 218, 255 });
        DrawSkyAtmosphere();

        BeginMode3D(camera);
        DrawPlane(V3(0.0f, -0.01f, 0.0f), (Vector2){ 18000.0f, 18000.0f }, (Color){ 55, 74, 58, 255 });
        DrawGroundGrid();

        DrawCircle3D(aircrafts[selectedAircraft].center, aircrafts[selectedAircraft].radius, V3(1.0f, 0.0f, 0.0f), 90.0f, Fade(SKYBLUE, 0.24f));
        DrawPrediction(missile, aircrafts[selectedAircraft], interceptPoint, interceptTime);
        DrawTrail(trail, YELLOW);
        for (int i = 0; i < MAX_AIRCRAFT; i++) {
            DrawAircraft(aircrafts[i], hasPlaneModel ? &planeModel : NULL);
        }
        DrawSphereWires(aircrafts[selectedAircraft].position, 14.0f, 12, 12, Fade(RED, 0.35f));

        if (missile.active || phase == PHASE_READY || phase == PHASE_MISSED) {
            DrawModelFacing(missileModel, missile.position, missile.forward, WHITE);
        }

        if (phase == PHASE_HIT) {
            DrawSphere(explosionPosition, 12.0f + resultTimer * 22.0f, Fade(ORANGE, 0.62f));
            DrawSphere(explosionPosition, 6.5f + resultTimer * 16.0f, Fade(YELLOW, 0.72f));
        }

        EndMode3D();
        DrawHud(phase, paused, timeScale, aircrafts[selectedAircraft], missile, interceptTime, telemetry, charts);
        DrawRadarScreen(aircrafts, MAX_AIRCRAFT, selectedAircraft, LAUNCH_POS);
        DrawFPS(GetScreenWidth() - 96, 16);
        EndDrawing();
    }

    UnloadTexture(missileTexture);
    if (hasPlaneModel) UnloadModel(planeModel);
    UnloadModel(missileModel);
    CloseWindow();
    return 0;
}
