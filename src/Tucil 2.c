#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define INITIAL_VERTEX_CAP 1024
#define INITIAL_FACE_CAP 1024
#define EPS 1e-7

typedef struct {
    double x, y, z;
} Vertex;

typedef struct {
    int a, b, c;
} Face;

typedef struct {
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;
} Box;

typedef struct OctreeNode {
    Box box;
    int depth;
    int is_leaf_voxel;
    struct OctreeNode *children[8];
} OctreeNode;

typedef struct {
    Vertex *data;
    int size;
    int cap;
} VertexArray;

typedef struct {
    Face *data;
    int size;
    int cap;
} FaceArray;

typedef struct {
    double x, y, z;
} Vec3;

static VertexArray g_vertices = {NULL, 0, 0};
static FaceArray g_faces = {NULL, 0, 0};

static long long *g_nodes_per_depth = NULL;
static long long *g_pruned_per_depth = NULL;
static int g_max_depth = 0;
static long long g_voxel_count = 0;

static void die(const char *msg) {
    fprintf(stderr, "Error: %s\n", msg);
    exit(EXIT_FAILURE);
}

static void *xmalloc(size_t sz) {
    void *p = malloc(sz);
    if (!p) die("alokasi memori gagal");
    return p;
}

static void *xrealloc(void *ptr, size_t sz) {
    void *p = realloc(ptr, sz);
    if (!p) die("realloc gagal");
    return p;
}

static int starts_with(const char *s, const char *prefix) {
    while (*prefix) {
        if (*s != *prefix) return 0;
        s++;
        prefix++;
    }
    return 1;
}

static void trim_newline(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) {
        s[n - 1] = '\0';
        n--;
    }
}

static int is_blank_or_comment(const char *line) {
    while (*line == ' ' || *line == '\t') line++;
    return (*line == '\0' || *line == '#');
}

static void init_vertex_array(VertexArray *arr) {
    arr->cap = INITIAL_VERTEX_CAP;
    arr->size = 0;
    arr->data = (Vertex *)xmalloc(arr->cap * sizeof(Vertex));
}

static void init_face_array(FaceArray *arr) {
    arr->cap = INITIAL_FACE_CAP;
    arr->size = 0;
    arr->data = (Face *)xmalloc(arr->cap * sizeof(Face));
}

static void push_vertex(VertexArray *arr, Vertex v) {
    if (arr->size >= arr->cap) {
        arr->cap *= 2;
        arr->data = (Vertex *)xrealloc(arr->data, arr->cap * sizeof(Vertex));
    }
    arr->data[arr->size++] = v;
}

static void push_face(FaceArray *arr, Face f) {
    if (arr->size >= arr->cap) {
        arr->cap *= 2;
        arr->data = (Face *)xrealloc(arr->data, arr->cap * sizeof(Face));
    }
    arr->data[arr->size++] = f;
}

static Vec3 v3(double x, double y, double z) {
    Vec3 v = {x, y, z};
    return v;
}

static Vec3 vec_sub(Vec3 a, Vec3 b) {
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static double vec_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 vec_cross(Vec3 a, Vec3 b) {
    return v3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

static double vec_abs(double x) {
    return x < 0 ? -x : x;
}

static void read_obj_file(const char *filepath) {
    FILE *fp = fopen(filepath, "r");
    if (!fp) die("tidak bisa membuka file input");

    init_vertex_array(&g_vertices);
    init_face_array(&g_faces);

    char line[1024];
    int line_no = 0;

    while (fgets(line, sizeof(line), fp)) {
        line_no++;
        trim_newline(line);

        if (is_blank_or_comment(line)) continue;

        while (*line == ' ' || *line == '\t') {
            memmove(line, line + 1, strlen(line));
        }

        if (starts_with(line, "v ")) {
            Vertex v;
            char extra[64];
            int n = sscanf(line, "v %lf %lf %lf %63s", &v.x, &v.y, &v.z, extra);
            if (n != 3) {
                fprintf(stderr, "Baris %d invalid untuk vertex: %s\n", line_no, line);
                die("format .obj tidak valid");
            }
            push_vertex(&g_vertices, v);
        } else if (starts_with(line, "f ")) {
            int a, b, c;
            char extra[64];
            int n = sscanf(line, "f %d %d %d %63s", &a, &b, &c, extra);
            if (n != 3) {
                fprintf(stderr, "Baris %d invalid untuk face: %s\n", line_no, line);
                die("format .obj tidak valid");
            }
            if (a <= 0 || b <= 0 || c <= 0) {
                fprintf(stderr, "Baris %d face index harus positif: %s\n", line_no, line);
                die("format .obj tidak valid");
            }
            Face f = {a - 1, b - 1, c - 1};
            push_face(&g_faces, f);
        } else {
            fprintf(stderr, "Baris %d tidak didukung: %s\n", line_no, line);
            die("format .obj tidak valid (hanya v x y z dan f i j k)");
        }
    }

    fclose(fp);

    if (g_vertices.size == 0) die("file .obj tidak memiliki vertex");
    if (g_faces.size == 0) die("file .obj tidak memiliki face");

    for (int i = 0; i < g_faces.size; i++) {
        Face f = g_faces.data[i];
        if (f.a < 0 || f.a >= g_vertices.size ||
            f.b < 0 || f.b >= g_vertices.size ||
            f.c < 0 || f.c >= g_vertices.size) {
            die("face mereferensikan vertex yang tidak ada");
        }
    }
}

static Box compute_mesh_bbox(void) {
    Box b;
    b.xmin = b.xmax = g_vertices.data[0].x;
    b.ymin = b.ymax = g_vertices.data[0].y;
    b.zmin = b.zmax = g_vertices.data[0].z;

    for (int i = 1; i < g_vertices.size; i++) {
        Vertex v = g_vertices.data[i];
        if (v.x < b.xmin) b.xmin = v.x;
        if (v.x > b.xmax) b.xmax = v.x;
        if (v.y < b.ymin) b.ymin = v.y;
        if (v.y > b.ymax) b.ymax = v.y;
        if (v.z < b.zmin) b.zmin = v.z;
        if (v.z > b.zmax) b.zmax = v.z;
    }

    return b;
}

static Box cube_from_bbox(Box mesh) {
    double dx = mesh.xmax - mesh.xmin;
    double dy = mesh.ymax - mesh.ymin;
    double dz = mesh.zmax - mesh.zmin;

    double side = dx;
    if (dy > side) side = dy;
    if (dz > side) side = dz;
    if (side < EPS) side = 1.0;

    double cx = (mesh.xmin + mesh.xmax) * 0.5;
    double cy = (mesh.ymin + mesh.ymax) * 0.5;
    double cz = (mesh.zmin + mesh.zmax) * 0.5;

    double pad = side * 1e-4;
    double h = side * 0.5 + pad;

    Box cube;
    cube.xmin = cx - h;
    cube.xmax = cx + h;
    cube.ymin = cy - h;
    cube.ymax = cy + h;
    cube.zmin = cz - h;
    cube.zmax = cz + h;

    return cube;
}

static int triangle_box_bbox_overlap(Box box, Vertex A, Vertex B, Vertex C) {
    double xmin = A.x, xmax = A.x;
    double ymin = A.y, ymax = A.y;
    double zmin = A.z, zmax = A.z;

    if (B.x < xmin) xmin = B.x;
    if (B.x > xmax) xmax = B.x;
    if (C.x < xmin) xmin = C.x;
    if (C.x > xmax) xmax = C.x;

    if (B.y < ymin) ymin = B.y;
    if (B.y > ymax) ymax = B.y;
    if (C.y < ymin) ymin = C.y;
    if (C.y > ymax) ymax = C.y;

    if (B.z < zmin) zmin = B.z;
    if (B.z > zmax) zmax = B.z;
    if (C.z < zmin) zmin = C.z;
    if (C.z > zmax) zmax = C.z;

    if (xmax < box.xmin - EPS || xmin > box.xmax + EPS) return 0;
    if (ymax < box.ymin - EPS || ymin > box.ymax + EPS) return 0;
    if (zmax < box.zmin - EPS || zmin > box.zmax + EPS) return 0;
    return 1;
}

static int axis_test(double p0, double p1, double p2, double r) {
    double minp = p0;
    double maxp = p0;
    if (p1 < minp) minp = p1;
    if (p1 > maxp) maxp = p1;
    if (p2 < minp) minp = p2;
    if (p2 > maxp) maxp = p2;
    if (minp > r + EPS || maxp < -r - EPS) return 0;
    return 1;
}

static int plane_box_overlap(Vec3 normal, Vec3 vert, Vec3 maxbox) {
    Vec3 vmin, vmax;

    if (normal.x > 0.0) {
        vmin.x = -maxbox.x - vert.x;
        vmax.x =  maxbox.x - vert.x;
    } else {
        vmin.x =  maxbox.x - vert.x;
        vmax.x = -maxbox.x - vert.x;
    }

    if (normal.y > 0.0) {
        vmin.y = -maxbox.y - vert.y;
        vmax.y =  maxbox.y - vert.y;
    } else {
        vmin.y =  maxbox.y - vert.y;
        vmax.y = -maxbox.y - vert.y;
    }

    if (normal.z > 0.0) {
        vmin.z = -maxbox.z - vert.z;
        vmax.z =  maxbox.z - vert.z;
    } else {
        vmin.z =  maxbox.z - vert.z;
        vmax.z = -maxbox.z - vert.z;
    }

    if (vec_dot(normal, vmin) > 0.0) return 0;
    if (vec_dot(normal, vmax) >= 0.0) return 1;
    return 0;
}

static int triangle_box_intersect_exact(Box box, Vertex A, Vertex B, Vertex C) {
    Vec3 center = v3(
        (box.xmin + box.xmax) * 0.5,
        (box.ymin + box.ymax) * 0.5,
        (box.zmin + box.zmax) * 0.5
    );

    Vec3 half = v3(
        (box.xmax - box.xmin) * 0.5,
        (box.ymax - box.ymin) * 0.5,
        (box.zmax - box.zmin) * 0.5
    );

    Vec3 v0 = v3(A.x - center.x, A.y - center.y, A.z - center.z);
    Vec3 v1 = v3(B.x - center.x, B.y - center.y, B.z - center.z);
    Vec3 v2 = v3(C.x - center.x, C.y - center.y, C.z - center.z);

    Vec3 e0 = vec_sub(v1, v0);
    Vec3 e1 = vec_sub(v2, v1);
    Vec3 e2 = vec_sub(v0, v2);

    double p0, p1, p2, r;

    p0 = e0.z * v0.y - e0.y * v0.z;
    p1 = e0.z * v1.y - e0.y * v1.z;
    p2 = e0.z * v2.y - e0.y * v2.z;
    r = vec_abs(e0.z) * half.y + vec_abs(e0.y) * half.z;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = -e0.z * v0.x + e0.x * v0.z;
    p1 = -e0.z * v1.x + e0.x * v1.z;
    p2 = -e0.z * v2.x + e0.x * v2.z;
    r = vec_abs(e0.z) * half.x + vec_abs(e0.x) * half.z;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = e0.y * v0.x - e0.x * v0.y;
    p1 = e0.y * v1.x - e0.x * v1.y;
    p2 = e0.y * v2.x - e0.x * v2.y;
    r = vec_abs(e0.y) * half.x + vec_abs(e0.x) * half.y;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = e1.z * v0.y - e1.y * v0.z;
    p1 = e1.z * v1.y - e1.y * v1.z;
    p2 = e1.z * v2.y - e1.y * v2.z;
    r = vec_abs(e1.z) * half.y + vec_abs(e1.y) * half.z;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = -e1.z * v0.x + e1.x * v0.z;
    p1 = -e1.z * v1.x + e1.x * v1.z;
    p2 = -e1.z * v2.x + e1.x * v2.z;
    r = vec_abs(e1.z) * half.x + vec_abs(e1.x) * half.z;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = e1.y * v0.x - e1.x * v0.y;
    p1 = e1.y * v1.x - e1.x * v1.y;
    p2 = e1.y * v2.x - e1.x * v2.y;
    r = vec_abs(e1.y) * half.x + vec_abs(e1.x) * half.y;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = e2.z * v0.y - e2.y * v0.z;
    p1 = e2.z * v1.y - e2.y * v1.z;
    p2 = e2.z * v2.y - e2.y * v2.z;
    r = vec_abs(e2.z) * half.y + vec_abs(e2.y) * half.z;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = -e2.z * v0.x + e2.x * v0.z;
    p1 = -e2.z * v1.x + e2.x * v1.z;
    p2 = -e2.z * v2.x + e2.x * v2.z;
    r = vec_abs(e2.z) * half.x + vec_abs(e2.x) * half.z;
    if (!axis_test(p0, p1, p2, r)) return 0;

    p0 = e2.y * v0.x - e2.x * v0.y;
    p1 = e2.y * v1.x - e2.x * v1.y;
    p2 = e2.y * v2.x - e2.x * v2.y;
    r = vec_abs(e2.y) * half.x + vec_abs(e2.x) * half.y;
    if (!axis_test(p0, p1, p2, r)) return 0;

    {
        double minv, maxv;

        minv = maxv = v0.x;
        if (v1.x < minv) minv = v1.x;
        if (v1.x > maxv) maxv = v1.x;
        if (v2.x < minv) minv = v2.x;
        if (v2.x > maxv) maxv = v2.x;
        if (minv > half.x + EPS || maxv < -half.x - EPS) return 0;

        minv = maxv = v0.y;
        if (v1.y < minv) minv = v1.y;
        if (v1.y > maxv) maxv = v1.y;
        if (v2.y < minv) minv = v2.y;
        if (v2.y > maxv) maxv = v2.y;
        if (minv > half.y + EPS || maxv < -half.y - EPS) return 0;

        minv = maxv = v0.z;
        if (v1.z < minv) minv = v1.z;
        if (v1.z > maxv) maxv = v1.z;
        if (v2.z < minv) minv = v2.z;
        if (v2.z > maxv) maxv = v2.z;
        if (minv > half.z + EPS || maxv < -half.z - EPS) return 0;
    }

    {
        Vec3 normal = vec_cross(e0, e1);
        if (!plane_box_overlap(normal, v0, half)) return 0;
    }

    return 1;
}

static int cube_intersects_any_face_exact(Box cube) {
    for (int i = 0; i < g_faces.size; i++) {
        Face f = g_faces.data[i];
        Vertex a = g_vertices.data[f.a];
        Vertex b = g_vertices.data[f.b];
        Vertex c = g_vertices.data[f.c];

        if (!triangle_box_bbox_overlap(cube, a, b, c)) continue;

        if (triangle_box_intersect_exact(cube, a, b, c)) {
            return 1;
        }
    }
    return 0;
}

static OctreeNode *create_node(Box box, int depth) {
    OctreeNode *node = (OctreeNode *)xmalloc(sizeof(OctreeNode));
    node->box = box;
    node->depth = depth;
    node->is_leaf_voxel = 0;
    for (int i = 0; i < 8; i++) node->children[i] = NULL;

    if (depth >= 0 && depth <= g_max_depth) {
        g_nodes_per_depth[depth]++;
    }

    return node;
}

static void subdivide_and_build(OctreeNode *node) {
    if (!cube_intersects_any_face_exact(node->box)) {
        if (node->depth >= 1 && node->depth <= g_max_depth) {
            g_pruned_per_depth[node->depth]++;
        }
        return;
    }

    if (node->depth == g_max_depth) {
        node->is_leaf_voxel = 1;
        g_voxel_count++;
        return;
    }

    double xmid = (node->box.xmin + node->box.xmax) * 0.5;
    double ymid = (node->box.ymin + node->box.ymax) * 0.5;
    double zmid = (node->box.zmin + node->box.zmax) * 0.5;

    int idx = 0;
    for (int dx = 0; dx < 2; dx++) {
        for (int dy = 0; dy < 2; dy++) {
            for (int dz = 0; dz < 2; dz++) {
                Box child;
                child.xmin = dx ? xmid : node->box.xmin;
                child.xmax = dx ? node->box.xmax : xmid;
                child.ymin = dy ? ymid : node->box.ymin;
                child.ymax = dy ? node->box.ymax : ymid;
                child.zmin = dz ? zmid : node->box.zmin;
                child.zmax = dz ? node->box.zmax : zmid;

                node->children[idx] = create_node(child, node->depth + 1);
                subdivide_and_build(node->children[idx]);
                idx++;
            }
        }
    }
}

static void free_octree(OctreeNode *node) {
    if (!node) return;
    for (int i = 0; i < 8; i++) {
        free_octree(node->children[i]);
    }
    free(node);
}

static void write_cube_obj(FILE *fp, Box b, long long *vertex_offset) {
    double x0 = b.xmin, x1 = b.xmax;
    double y0 = b.ymin, y1 = b.ymax;
    double z0 = b.zmin, z1 = b.zmax;

    fprintf(fp, "v %f %f %f\n", x0, y0, z0);
    fprintf(fp, "v %f %f %f\n", x1, y0, z0);
    fprintf(fp, "v %f %f %f\n", x1, y1, z0);
    fprintf(fp, "v %f %f %f\n", x0, y1, z0);
    fprintf(fp, "v %f %f %f\n", x0, y0, z1);
    fprintf(fp, "v %f %f %f\n", x1, y0, z1);
    fprintf(fp, "v %f %f %f\n", x1, y1, z1);
    fprintf(fp, "v %f %f %f\n", x0, y1, z1);

    long long s = *vertex_offset + 1;

    fprintf(fp, "f %lld %lld %lld\n", s + 0, s + 1, s + 2);
    fprintf(fp, "f %lld %lld %lld\n", s + 0, s + 2, s + 3);

    fprintf(fp, "f %lld %lld %lld\n", s + 4, s + 6, s + 5);
    fprintf(fp, "f %lld %lld %lld\n", s + 4, s + 7, s + 6);

    fprintf(fp, "f %lld %lld %lld\n", s + 0, s + 4, s + 5);
    fprintf(fp, "f %lld %lld %lld\n", s + 0, s + 5, s + 1);

    fprintf(fp, "f %lld %lld %lld\n", s + 3, s + 2, s + 6);
    fprintf(fp, "f %lld %lld %lld\n", s + 3, s + 6, s + 7);

    fprintf(fp, "f %lld %lld %lld\n", s + 0, s + 3, s + 7);
    fprintf(fp, "f %lld %lld %lld\n", s + 0, s + 7, s + 4);

    fprintf(fp, "f %lld %lld %lld\n", s + 1, s + 5, s + 6);
    fprintf(fp, "f %lld %lld %lld\n", s + 1, s + 6, s + 2);

    *vertex_offset += 8;
}

static void export_voxels_dfs(FILE *fp, OctreeNode *node, long long *vertex_offset) {
    if (!node) return;

    if (node->is_leaf_voxel) {
        write_cube_obj(fp, node->box, vertex_offset);
        return;
    }

    for (int i = 0; i < 8; i++) {
        export_voxels_dfs(fp, node->children[i], vertex_offset);
    }
}

static void write_output_obj(const char *output_path, OctreeNode *root) {
    FILE *fp = fopen(output_path, "w");
    if (!fp) die("tidak bisa membuat file output .obj");

    fprintf(fp, "# Hasil voxelisasi\n");
    long long vertex_offset = 0;
    export_voxels_dfs(fp, root, &vertex_offset);

    fclose(fp);
}

static char *build_output_path(const char *input_path) {
    const char *filename = strrchr(input_path, '/');
    if (!filename) filename = strrchr(input_path, '\\');
    if (!filename) filename = input_path;
    else filename++;

    char name[256];
    strcpy(name, filename);

    char *dot = strrchr(name, '.');
    if (dot) *dot = '\0';

    char *out = (char *)xmalloc(512);
    sprintf(out, "../test/%s-hasil.obj", name);
    return out;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input.obj> <max_depth>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char *input_path = argv[1];
    g_max_depth = atoi(argv[2]);
    if (g_max_depth < 1) {
        fprintf(stderr, "max_depth harus >= 1\n");
        return EXIT_FAILURE;
    }

    clock_t start = clock();

    g_nodes_per_depth = (long long *)calloc((size_t)g_max_depth + 1, sizeof(long long));
    g_pruned_per_depth = (long long *)calloc((size_t)g_max_depth + 1, sizeof(long long));
    if (!g_nodes_per_depth || !g_pruned_per_depth) die("calloc gagal");

    read_obj_file(input_path);

    Box mesh_bbox = compute_mesh_bbox();
    Box root_cube = cube_from_bbox(mesh_bbox);

    OctreeNode *root = create_node(root_cube, 0);
    subdivide_and_build(root);

    char *output_path = build_output_path(input_path);
    write_output_obj(output_path, root);

    clock_t end = clock();
    double elapsed_sec = (double)(end - start) / CLOCKS_PER_SEC;

    long long out_vertices = g_voxel_count * 8LL;
    long long out_faces = g_voxel_count * 12LL;

    printf("HASIL :\n");
    printf("Banyaknya voxel yang terbentuk   : %lld\n", g_voxel_count);
    printf("Banyaknya vertex yang terbentuk  : %lld\n", out_vertices);
    printf("Banyaknya faces yang terbentuk   : %lld\n", out_faces);

    printf("\nStatistik node octree yang terbentuk:\n");
    for (int d = 1; d <= g_max_depth; d++) {
        printf("%d : %lld\n", d, g_nodes_per_depth[d]);
    }

    printf("\nStatistik node yang tidak perlu ditelusuri:\n");
    for (int d = 1; d <= g_max_depth; d++) {
        printf("%d : %lld\n", d, g_pruned_per_depth[d]);
    }

    printf("\nKedalaman octree                : %d\n", g_max_depth);
    printf("Lama waktu program berjalan     : %f detik\n", elapsed_sec);
    printf("Path file .obj hasil            : %s\n", output_path);

    free(output_path);
    free_octree(root);
    free(g_nodes_per_depth);
    free(g_pruned_per_depth);
    free(g_vertices.data);
    free(g_faces.data);

    return EXIT_SUCCESS;
}