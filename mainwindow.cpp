#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    clear_draw_buffer = QImage(SCREEN_WIDTH, SCREEN_HEIGHT, QImage::Format_RGB32);
    clear_draw_buffer.fill(Qt::black);
    clear_z_buffer = QVector<QVector<qreal>>(SCREEN_WIDTH, QVector<qreal>(SCREEN_HEIGHT, std::numeric_limits<qreal>::infinity()));
    current_shading = WIREFRAME;
    camera_pos = {0, 0, 5};
    camera_angles = {0, 0, 0};
    const qreal a = 3, d = 1;
    point_lights = {{{0, a, d}, QColor(255, 0, 0)},
                   {{-a / 2, -qSqrt(3) / 2 * a, d}, QColor(0, 255, 0)},
                   {{a / 2, -qSqrt(3) / 2 * a, d}, QColor(0, 0, 255)}};
    auto temp = Cube(2, Qt::white);
    temp.translate_cube(0, 0, 7);
    objects.push_back(temp);
    selected_object = &objects[0];
}

MainWindow::~MainWindow()
{
    delete ui;
}

QVector<QVector<qreal>> MainWindow::rotation_matrix(QVector<qreal> axis, qreal theta){
    auto temp = qSqrt(dot(axis, axis));
    for (auto &i: axis) {
        i /= temp;
    }
    auto a = qCos(theta / 2.0);
    temp = qSin(theta / 2.0);
    auto b = -axis[0] * temp;
    auto c = -axis[1] * temp;
    auto d = -axis[2] * temp;
    auto aa = a * a;
    auto bb = b * b;
    auto cc = c * c;
    auto dd = d * d;
    auto bc = b * c;
    auto ad = a * d;
    auto ac = a * c;
    auto ab = a * b;
    auto bd = b * d;
    auto cd = c * d;
    return QVector<QVector<qreal>>{{aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)},
                                   {2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)},
                                   {2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc}};
}

qreal MainWindow::dot(const QVector<qreal> &a, const QVector<qreal> &b){
    auto temp = vector_multiplication(a, b);
    return std::accumulate(temp.begin(), temp.end(), (qreal)0.0);
}

QVector<qreal> MainWindow::dot(const QVector<QVector<qreal>> &matrix, const QVector<qreal> &vector){
    QVector<qreal> result(vector.size(), 0);
    std::transform(std::execution::unseq, matrix.begin(), matrix.end(), result.begin(), [&](const auto &i){
        auto temp = vector_multiplication(i, vector);
        return std::accumulate(temp.begin(), temp.end(), (qreal)0.0);
    });
    return result;
}

QVector<qreal> MainWindow::dot(const QVector<qreal> &vector, const QVector<QVector<qreal>> &matrix){
    QVector<qreal> result(matrix[0].size(), 0);
    for (qsizetype i = 0; i < matrix.size(); i++) {
        std::transform(std::execution::unseq, matrix[i].begin(), matrix[i].end(), result.begin(), result.begin(), [&](const auto &f, const auto &s){
            return s + f * vector[i];
        });
    }
    return result;
}

MainWindow::Point3D::Point3D(const qreal x, const qreal y, const qreal z){
    this->x = x;
    this->y = y;
    this->z = z;
}

MainWindow::Point3D MainWindow::Point3D::operator-(const Point3D &other) const{
    return Point3D(x - other.x, y - other.y, z - other.z);
}

QVector<qreal> MainWindow::Point3D::to_arr() const{
    return QVector<qreal>{x, y, z};
}

QPair<qreal, qreal> MainWindow::Point3D::determ_sincos(qreal angle) const{
    auto rad = qDegreesToRadians(angle);
    return qMakePair(qSin(rad), qCos(rad));
}

MainWindow::Point3D MainWindow::Point3D::rotate_X(qreal angle) const{
    auto temp = determ_sincos(angle);
    auto temp_y = y * temp.second - z * temp.first;
    auto temp_z = y * temp.first + z * temp.second;
    return Point3D(x, temp_y, temp_z);
}

MainWindow::Point3D MainWindow::Point3D::rotate_Y(qreal angle) const{
    auto temp = determ_sincos(angle);
    auto temp_z = z * temp.second - x * temp.first;
    auto temp_x = z * temp.first + x * temp.second;
    return Point3D(temp_x, y, temp_z);
}

MainWindow::Point3D MainWindow::Point3D::rotate_Z(qreal angle) const{
    auto temp = determ_sincos(angle);
    auto temp_x = x * temp.second - y * temp.first;
    auto temp_y = x * temp.first + y * temp.second;
    return Point3D(temp_x, temp_y, z);
}

MainWindow::Point3D MainWindow::Point3D::rotate(qreal a_x, qreal a_y, qreal a_z) const{
    a_x = qDegreesToRadians(a_x);
    a_y = qDegreesToRadians(a_y);
    a_z = qDegreesToRadians(a_z);
    auto sin_a = qSin(a_x);
    auto cos_a = qCos(a_x);
    auto sin_b = qSin(a_y);
    auto cos_b = qCos(a_y);
    auto sin_g = qSin(a_z);
    auto cos_g = qCos(a_z);

    QVector<QVector<qreal>> matrix = {{cos_b * cos_g , sin_a * sin_b * cos_g - cos_a * sin_g, cos_a * sin_b * cos_g + sin_a * sin_g},
                                      {cos_b * sin_g, sin_a * sin_b * sin_g + cos_a * cos_g, cos_a * sin_b * sin_g - sin_a * cos_g},
                                      {-sin_b, sin_a * cos_b, cos_a * cos_b}};

    auto temp = dot(matrix, {x, y, z});
    return Point3D(temp[0], temp[1], temp[2]);
}

MainWindow::Point3D MainWindow::Point3D::rotate_yx(qreal a_x, qreal a_y) const{
    QVector<QVector<qreal>> x_matrix = {{1, 0, 0, 0},
                                        {0, 1, 0, 0},
                                        {0, 0, 1, 0},
                                        {0, 0, 0, 1}};
    auto temp = rotation_matrix({1, 0, 0}, a_x);
    for (qsizetype i = 0; i<3; i++) {
        for (qsizetype j = 0; j<3; j++) {
            x_matrix[i][j] = temp[i][j];
        }
    }
    QVector<QVector<qreal>> y_matrix = {{1, 0, 0, 0},
                                        {0, 1, 0, 0},
                                        {0, 0, 1, 0},
                                        {0, 0, 0, 1}};
    temp = rotation_matrix({0, 1, 0}, a_y);
    for (qsizetype i = 0; i < 3; i++) {
        for (qsizetype j = 0; j < 3; j++) {
            y_matrix[i][j] = temp[i][j];
        }
    }

    auto result = dot(x_matrix, dot(y_matrix, {x, y, z, 1}));
    return Point3D(result[0], result[1], result[2]);
}

MainWindow::Point3D MainWindow::Point3D::rotate_xy(qreal a_x, qreal a_y) const{
    QVector<QVector<qreal>> x_matrix = {{1, 0, 0, 0},
                                        {0, 1, 0, 0},
                                        {0, 0, 1, 0},
                                        {0, 0, 0, 1}};
    auto temp = rotation_matrix({1, 0, 0}, a_x);
    for (qsizetype i = 0; i < 3; i++) {
        for (qsizetype j = 0; j < 3; j++) {
            x_matrix[i][j] = temp[i][j];
        }
    }
    QVector<QVector<qreal>> y_matrix = {{1, 0, 0, 0},
                                        {0, 1, 0, 0},
                                        {0, 0, 1, 0},
                                        {0, 0, 0, 1}};
    temp = rotation_matrix({0, 1, 0}, a_y);
    for (qsizetype i = 0; i < 3; i++) {
        for (qsizetype j = 0; j < 3; j++) {
            y_matrix[i][j] = temp[i][j];
        }
    }

    auto result = dot(y_matrix, dot(x_matrix, {x, y, z, 1}));
    return Point3D(result[0], result[1], result[2]);
}

MainWindow::Point3D MainWindow::Point3D::project() const{
    qreal temp_x, temp_y;
    if (z > 0){
        auto factor = FOCAL_LENGTH / z;
        temp_x = x * factor + SCREEN_WIDTH / 2;
        temp_y = y * factor + SCREEN_HEIGHT / 2;
    } else {
        temp_x = temp_y = 0;
    }
    return Point3D(temp_x, temp_y, z);
}

MainWindow::PointLight::PointLight(const Point3D &point, const QColor &color): point(point){
    this->color = color;
}

void MainWindow::PointLight::translate_cube(const qreal x, const qreal y, const qreal z){
    point.x += x;
    point.y += y;
    point.z += z;
}

QPair<QVector<qreal>, QColor> MainWindow::PointLight::to_arr() const{
    return qMakePair(point.to_arr(), color);
}

QVector<qreal> MainWindow::crossProduct(const QVector<qreal> &a, const QVector<qreal> &b){
    QVector<qreal> result(3);
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
    return result;
}

MainWindow::Point3D MainWindow::crossProduct(const Point3D &a, const Point3D &b){
    auto temp = crossProduct(a.to_arr(), b.to_arr());
    return Point3D(temp[0], temp[1], temp[2]);
}

template <class T>
QVector<QVector<T>> MainWindow::vector_minus_mat(const QVector<T> &a, const QVector<QVector<T>> &b){
    QVector<QVector<T>> result(b.size());
    std::transform(std::execution::par_unseq, b.begin(), b.end(), result.begin(), [&](QVector<T> i){
        std::transform(std::execution::unseq, a.begin(), a.end(), i.begin(), i.begin(), [&](const T &k, const T &l){
            return k - l;
        });
        return i;
    });
    return result;
}

QVector<QVector<qreal>> MainWindow::vector_minus_mat(const QVector<qreal> &a, const QVector<Point3D> &b){
    QVector<QVector<qreal>> result(b.size());
    std::transform(std::execution::par_unseq, b.begin(), b.end(), result.begin(), [&](const Point3D &i){
        return QVector<qreal>{a[0] - i.x, a[1] - i.y, a[2] - i.z};
    });
    return result;
}

qreal MainWindow::linalg_norm(const QVector<qreal> &x){
    qreal result = 0;
    for (auto &i: x) {
        result += qPow(i, 2);
    }
    return qSqrt(result);
}

QVector<qreal> MainWindow::linspace(qreal x1, qreal x2, int num){
    QVector<qreal> result;
    qreal step = (x2 - x1) / num;
    for (int i = 0;  i < num - 1; i++) {
        result.push_back(x1 + i * step);
    }
    result.push_back(x2);
    return result;
}

template <class T>
QVector<T> MainWindow::vector_division(QVector<T> a, const T &b){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](T &i){
        i /= b;
    });
    return a;   
}

template <class T>
QVector<T> MainWindow::vector_division(const T &b, QVector<T> a){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](T &i){
        i = b / i;
    });
    return a;
}

template <class T>
QVector<T> MainWindow::vector_division(const QVector<T> &a, const QVector<T> &b){
    auto result = QVector<T>(qMin(a.size(), b.size()));
    std::transform(std::execution::unseq, a.begin(), a.end(), b.begin(), result.begin(), [&](const T &i, const T &j){
        return i / j;
    });
    return result;
}

template <class T>
QVector<T> MainWindow::vector_sum(QVector<T> a, const T &b){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](T &i){
        i += b;
    });
    return a;
}

template <class T>
QVector<T> MainWindow::vector_sum(const QVector<T> &a, const QVector<T> &b){
    auto result = QVector<T>(qMin(a.size(), b.size()));
    std::transform(std::execution::unseq, a.begin(), a.end(), b.begin(), result.begin(), [&](const T &i, const T &j){
        return i + j;
    });
    return result;
}

template <class T>
QVector<T> MainWindow::vector_multiplication(QVector<T> a, const T &b){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](T &i){
        i *= b;
    });
    return a;
}

template <class T>
QVector<T> MainWindow::vector_multiplication(const QVector<T> &a, const QVector<T> &b){
    auto result = QVector<T>(qMin(a.size(), b.size()));
    std::transform(std::execution::unseq, a.begin(), a.end(), b.begin(), result.begin(), [&](const T &i, const T &j){
        return i * j;
    });
    return result;
}

template <class T>
QVector<T> MainWindow::vector_minus(QVector<T> a, const T &b){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](T &i){
        i -= b;
    });
    return a;
}

template <class T>
QVector<T> MainWindow::vector_minus(const T &b, QVector<T> a){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](T &i){
        i = b - i;
    });
    return a;
}

template <class T>
QVector<T> MainWindow::vector_minus(const QVector<T> &a, const QVector<T> &b){
    auto result = QVector<T>(qMin(a.size(), b.size()));
    std::transform(std::execution::unseq, a.begin(), a.end(), b.begin(), result.begin(), [&](const T &i, const T &j){
        return i - j;
    });
    return result;
}

QVector<qreal> MainWindow::vector_abs(QVector<qreal> a){
    std::for_each(std::execution::unseq, a.begin(), a.end(), [&](qreal &i){
        i = qFabs(i);
    });
    return a;
}

QColor MainWindow::color_multiplication(const QColor &a, const QColor &b){
    float a_r, a_g, a_b, b_r, b_g, b_b;
    a.getRgbF(&a_r, &a_g, &a_b);
    b.getRgbF(&b_r, &b_g, &b_b);
    float result_r, result_g, result_b;
    result_r = a_r * b_r;
    result_g = a_g * b_g;
    result_b = a_b * b_b;
    result_r = result_r > 1 ? 1 : result_r;
    result_g = result_g > 1 ? 1 : result_g;
    result_b = result_b > 1 ? 1 : result_b;
    result_r = result_r < 0 ? 0 : result_r;
    result_g = result_g < 0 ? 0 : result_g;
    result_b = result_b < 0 ? 0 : result_b;
    return QColor(result_r * 255, result_g * 255, result_b * 255);
}

QColor MainWindow::color_multiplication(const QColor &a, const qreal &b){
    float a_r, a_g, a_b;
    a.getRgbF(&a_r, &a_g, &a_b);
    float result_r, result_g, result_b;
    result_r = a_r * b;
    result_g = a_g * b;
    result_b = a_b * b;
    result_r = result_r > 1 ? 1 : result_r;
    result_g = result_g > 1 ? 1 : result_g;
    result_b = result_b > 1 ? 1 : result_b;
    result_r = result_r < 0 ? 0 : result_r;
    result_g = result_g < 0 ? 0 : result_g;
    result_b = result_b < 0 ? 0 : result_b;
    return QColor(result_r * 255, result_g * 255, result_b * 255);
}

QColor MainWindow::color_sum(const QColor &a, const QColor &b){
    float a_r, a_g, a_b, b_r, b_g, b_b;
    a.getRgbF(&a_r, &a_g, &a_b);
    b.getRgbF(&b_r, &b_g, &b_b);
    float result_r, result_g, result_b;
    result_r = a_r + b_r;
    result_g = a_g + b_g;
    result_b = a_b + b_b;
    result_r = result_r > 1 ? 1 : result_r;
    result_g = result_g > 1 ? 1 : result_g;
    result_b = result_b > 1 ? 1 : result_b;
    result_r = result_r < 0 ? 0 : result_r;
    result_g = result_g < 0 ? 0 : result_g;
    result_b = result_b < 0 ? 0 : result_b;
    return QColor(result_r * 255, result_g * 255, result_b * 255);
}

template <class T>
QVector<QVector<T>> MainWindow::matrix_transpose(const QVector<QVector<T>> &source){
    QVector<QVector<T>> result(source[0].size(), QVector<T>(source.size()));
    for (qsizetype i = 0; i < source.size(); i++) {
        for (qsizetype j = 0; j < source[i].size(); j++) {
            result[j][i] = source[i][j];
        }
    }
    return result;
}

template <class T>
QVector<QVector<T>> MainWindow::matrix_multiplication(const QVector<QVector<T>> &a, const QVector<QVector<T>> &b){
    QVector<QVector<T>> result(a.size(), QVector<T>(b[0].size(), T()));
    for (qsizetype i = 0; i < result.size(); i++) {
        for (qsizetype k = 0; k < a[0].size(); k++) {
            for (qsizetype j = 0; j < result[i].size(); j++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return result;
}

QPair<QVector<QVector<qreal>>, QVector<QVector<qreal>>> MainWindow::get_c_and_norm(const QVector<QVector<int>> &faces, const QVector<Point3D> &vertices, bool return_planes){
    QVector<QVector<qreal>> normals, centers;
    for (auto i=0; i<faces.size(); i++) {
        auto &face = faces[i];
        qreal temp_x = 0, temp_y = 0, temp_z = 0;
        for (auto &j: face) {
           temp_x+=vertices[j].x;
           temp_y+=vertices[j].y;
           temp_z+=vertices[j].z;
        }
        auto center = QVector<qreal>{temp_x / 4, temp_y / 4, temp_z / 4};
        auto p1 = vertices[face[0]];
        auto p2 = vertices[face[1]];
        auto p3 = vertices[face[2]];
        auto v1 = p3 - p1;
        auto v2 = p2 - p1;
        auto normal = crossProduct(v1, v2).to_arr();
        for (auto &j: normal) {
            j *= -1;
        }

        if (return_planes){
            auto d = dot(normal, p1.to_arr());
            normal.push_back(d);
        }
        normals.push_back(normal);
        centers.push_back(center);
    }

    return qMakePair(centers, normals);
}

QVector<QColor> MainWindow::lambert_lighting(const QVector<QVector<int>> &faces, const QVector<Point3D> &vertices, const QVector<PointLight> &point_lights, const QColor &color){
    QVector<QColor> intensities(faces.size(), QColor(0, 0, 0));
    const auto &[centers, normals] = get_c_and_norm(faces, vertices);
    for (auto &light: point_lights) {
        const auto &[light_pos, light_color] = light.to_arr();
        auto light_vectors = vector_minus_mat(light_pos, centers);
        for (qsizetype i = 0; i < light_vectors.size(); i++) {
            auto light_vector = vector_division(light_vectors[i], linalg_norm(light_vectors[i]));
            auto normal = vector_division(normals[i], linalg_norm(normals[i]));
            auto intensity_coef = dot(light_vector, normal);
            intensity_coef = intensity_coef > 0 ? intensity_coef : 0;
            intensities[i] = color_sum(color_multiplication(color_multiplication(color, light_color), intensity_coef), intensities[i]);
        }
    }
    return intensities;
}

QVector<QColor> MainWindow::gouraud_lighting(const QVector<QVector<int>> &faces, const QVector<Point3D> &vertices, const QVector<QVector<int>> &vertices_faces, const QVector<PointLight> &point_lights, const QColor &color){
    QVector<QColor> intensities(vertices.size(), QColor(0, 0, 0));
    const auto &[_, faces_normals] = get_c_and_norm(faces, vertices);
    auto vertices_normals = QVector<QVector<qreal>>(vertices.size(), QVector<qreal>(3, 0));
    for (qsizetype i = 0; i < vertices_faces.size(); i++) {
        auto &faces = vertices_faces[i];
        for (auto &v: faces) {
            vertices_normals[i] = vector_sum(vertices_normals[i], faces_normals[v]);
        }
        vertices_normals[i] = vector_division(vertices_normals[i], (qreal)faces.size());
    }
    for (auto &light: point_lights){
        const auto &[light_pos, light_color] = light.to_arr();
        auto light_vectors = vector_minus_mat(light_pos, vertices);
        for (qsizetype i = 0; i < light_vectors.size(); i++){
            auto light_vector = vector_division(light_vectors[i], linalg_norm(light_vectors[i]));
            auto normal = vector_division(vertices_normals[i], linalg_norm(vertices_normals[i]));
            auto intensity_coef = dot(light_vector, normal);
            intensity_coef = intensity_coef > 0 ? intensity_coef : 0;
            intensities[i] = color_sum(color_multiplication(color_multiplication(color, light_color), intensity_coef), intensities[i]);
        }
    }
    return intensities;
}

MainWindow::Cube::Cube(qreal size, QColor color){
    vertices = {Point3D(-1 * size, 1 * size, -1 * size),
                Point3D(1 * size, 1 * size, -1 * size),
                Point3D(1 * size, -1 * size, -1 * size),
                Point3D(-1 * size, -1 * size, -1 * size),
                Point3D(-1 * size, 1 * size, 1 * size),
                Point3D(1 * size, 1 * size, 1 * size),
                Point3D(1 * size, -1 * size, 1 * size),
                Point3D(-1 * size, -1 * size, 1 * size)};
    faces = {{0, 1, 2, 3},
             {1, 5, 6, 2},
             {5, 4, 7, 6},
             {4, 0, 3, 7},
             {0, 4, 5, 1},
             {3, 2, 6, 7}};
    vertices_faces = QVector<QVector<int>>(vertices.size());
    for (qsizetype i = 0; i < faces.size(); i++)
        for (auto &j: faces[i]) {
            vertices_faces[j].push_back(i);
        }
    angles = {0, 0, 0};
    pos = {0, 0, 0};
    this->color = color;
}

QVector<MainWindow::Point3D> MainWindow::Cube::transform_vertices(const QVector<qreal> &camera_pos, const QVector<qreal> &camera_angles, bool project) const{
    QVector<Point3D> result = vertices;

    std::for_each(std::execution::unseq, result.begin(), result.end(), [&](auto &vertex){
        vertex = vertex.rotate(angles[0], angles[1], angles[2]);
        vertex.x += camera_pos[0] + pos[0];
        vertex.y += camera_pos[1] + pos[1];
        vertex.z += camera_pos[2] + pos[2];

        vertex = vertex.rotate_yx(camera_angles[0], camera_angles[1]);

        if (project){
            vertex = vertex.project();
        }
    });
    return result;
}

QVector<QPair<qsizetype, qreal>> MainWindow::Cube::calculate_avg_z(const QVector<Point3D> &vertices) const{
    QVector<QPair<qsizetype, qreal>> avg_z(faces.size());
    for (qsizetype i = 0; i < faces.size(); i++) {
        auto &face = faces[i];
        auto z = (vertices[face[0]].z +
                vertices[face[1]].z +
                vertices[face[2]].z +
                vertices[face[3]].z) / 4.0;
        avg_z.push_back(qMakePair(i, z));
    }
    return avg_z;
}

QPair<QVector<QVector<qreal>>, QVector<QVector<qreal>>> MainWindow::Cube::get_c_and_norm(const QVector<Point3D> &transformed_vertices, bool return_d) const{
    return MainWindow::get_c_and_norm(faces, transformed_vertices, return_d);
}

void MainWindow::Cube::translate_cube(const qreal x, const qreal y, const qreal z){
    pos[0] += x;
    pos[1] += y;
    pos[2] += z;
}

void MainWindow::Cube::rotate_cube(Direction direction, qreal speed){
    switch (direction) {
    case UP:
        angles[0] += 2 * speed;
        break;
    case DOWN:
        angles[0] -= 2 * speed;
        break;
    case LEFT:
        angles[1] += 2 * speed;
        break;
    case RIGHT:
        angles[1] -= 2 * speed;
        break;
    case FORWARD:
        angles[2] += 2 * speed;
        break;
    case BACKWARDS:
        angles[2] -= 2 * speed;
        break;
    default:
        break;
    }
}

void MainWindow::Cube::lambert_make_color(const QVector<PointLight> &point_lights){
    if (point_lights.empty())
        return;
    const auto &t_vertices = transform_vertices({0, 0, 0}, {0, 0, 0}, false);
    lambert_colors = lambert_lighting(faces, t_vertices, point_lights, color);
}

void MainWindow::Cube::gouraud_make_color(const QVector<PointLight> &point_lights){
    if (point_lights.empty())
        return;
    const auto &t_vertices = transform_vertices({0, 0, 0}, {0, 0, 0}, false);
    gouraud_colors = gouraud_lighting(faces, t_vertices, vertices_faces, point_lights, color);
}

QVector<QPair<int, QVector<QPair<int, QVector<qreal>>>>> MainWindow::Cube::draw_cube(const QVector<qreal> &camera_pos, const QVector<qreal> &camera_angles, const QVector<PointLight> &point_lights, Shading shading, bool triangles) {
    const auto &t_vertices = transform_vertices(camera_pos, camera_angles, true);
    auto avg_Z = calculate_avg_z(t_vertices);

    QVector<QPair<int, QVector<QPair<int, QVector<qreal>>>>> polygons;

    auto [_, normals] = get_c_and_norm(t_vertices, true);

    auto vis = dot({0, 0, -1, 0}, matrix_transpose(normals));

    std::sort(avg_Z.begin(), avg_Z.end(), [](auto &left, auto &right) {
        return left.second > right.second;
    });
    for (auto &z_val: avg_Z) {
        if (vis[z_val.first] <= 0)
            continue;
        if (z_val.second < 3)
            continue;
        auto f_index = z_val.first;
        auto &f = faces[f_index];

        if (triangles){
            QVector<QPair<int, QVector<qreal>>> point_list = {
                {f[0], t_vertices[f[0]].to_arr()},
                {f[1], t_vertices[f[1]].to_arr()},
                {f[2], t_vertices[f[2]].to_arr()}
            };
            polygons.push_back({f_index, point_list});

            point_list = {
                {f[0], t_vertices[f[0]].to_arr()},
                {f[2], t_vertices[f[2]].to_arr()},
                {f[3], t_vertices[f[3]].to_arr()}
            };
            polygons.push_back({f_index, point_list});
        } else {
            QVector<QPair<int, QVector<qreal>>> point_list = {
                {f[0], t_vertices[f[0]].to_arr()},
                {f[1], t_vertices[f[1]].to_arr()},
                {f[2], t_vertices[f[2]].to_arr()},
                {f[3], t_vertices[f[3]].to_arr()}
            };
            polygons.push_back({f_index, point_list});
        }
    }

    if (polygons.size() > 0){
        switch (shading) {
        case LAMBERT:
            lambert_make_color(point_lights);
            break;

        case GOURAUD:
            gouraud_make_color(point_lights);
            break;

        default:
            break;
        }
    }

    return polygons;
}

qreal MainWindow::area(qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3){
    return qFabs((x1 * (y2 - y3) + x2 * (y3 - y1)
                   + x3 * (y1 - y2)) / 2.0);
}

QVector<qreal> MainWindow::area(const QVector<qreal> &x1, const QVector<qreal> &y1, qreal x2, qreal y2, qreal x3, qreal y3){
    auto y2_minus_y3 = y2 - y3;
    auto y3_minus_y1 = vector_minus(y3, y1);
    auto y1_minus_y2 = vector_minus(y1, y2);
    auto x1_multiple_y2_minus_y3 = vector_multiplication(x1, y2_minus_y3);
    auto x2_multiple_y3_minus_y1 = vector_multiplication(y3_minus_y1, x2);
    auto x3_multiple_y1_minus_y2 = vector_multiplication(y1_minus_y2, x3);
    auto x1_multiple_y2_minus_y3_sum_x2_multiple_y3_minus_y1 = vector_sum(x1_multiple_y2_minus_y3, x2_multiple_y3_minus_y1);
    auto common_sum = vector_sum(x1_multiple_y2_minus_y3_sum_x2_multiple_y3_minus_y1, x3_multiple_y1_minus_y2);
    return vector_abs(vector_division(common_sum, 2.0));
}

QVector<qreal> MainWindow::area(qreal x1, qreal y1, const QVector<qreal> &x2, const QVector<qreal> &y2, qreal x3, qreal y3){
    return area(x2, y2, x1, y1, x3, y3);
}

QVector<qreal> MainWindow::area(qreal x1, qreal y1, qreal x2, qreal y2, const QVector<qreal> &x3, const QVector<qreal> &y3){
    return area(x3, y3, x1, y1, x2, y2);
}

std::tuple<QVector<bool>, QVector<qreal>, QVector<qreal>, QVector<qreal>> MainWindow::is_inside_triangle(const QVector<qreal> &x, const QVector<qreal> &y, qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3){
    auto A = area(x1, y1, x2, y2, x3, y3);
    auto A1 = area(x, y, x2, y2, x3, y3);
    auto A2 = area(x1, y1, x, y, x3, y3);
    auto A3 = area(x1, y1, x2, y2, x, y);
    auto temp_v = vector_minus(vector_sum(A1, vector_sum(A2, A3)), A);
    QVector<bool> bool_result(temp_v.size());
    std::transform(std::execution::unseq, temp_v.begin(), temp_v.end(), bool_result.begin(), [&](const qreal &i){
        return i < 0.001;
    });
    return std::make_tuple(bool_result, vector_division(A1, A), vector_division(A2, A), vector_division(A3, A));
}

std::tuple<QVector<qreal>, QVector<qreal>, QVector<QVector<qreal>>> MainWindow::get_triangle_points(qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3){
    const QVector<qreal> xs = {x1, x2, x3};
    const QVector<qreal> ys = {y1, y2, y3};
    int x_min = *std::min_element(xs.begin(), xs.end());
    int y_min = *std::min_element(ys.begin(), ys.end());
    int x_max = *std::max_element(xs.begin(), xs.end());
    int y_max = *std::max_element(ys.begin(), ys.end());
    int x_delta = x_max - x_min + 1;
    int y_delta = y_max - y_min + 1;

    if (x_max < 0 || x_min > SCREEN_WIDTH || y_max < 0 || y_min > SCREEN_HEIGHT){
        x_delta = y_delta = 1;
    } else {
        if (x_min == x_max || y_min == y_max)
            x_delta = y_delta = 1;
    }
    QVector<qreal> X(x_delta * y_delta), Y(x_delta * y_delta);
    for (qsizetype i = 0; i < x_delta; i++) {
        for (qsizetype j = 0; j < y_delta; j++) {
            X[i * y_delta + j] = i;
            Y[i * y_delta + j] = j;
        }
    }
    {
        QVector<bool> screen_mask((qsizetype)(x_delta * y_delta));
        auto temp_x = vector_sum(X, (qreal)x_min);
        auto temp_y = vector_sum(Y, (qreal)y_min);
        std::transform(std::execution::unseq, temp_x.begin(), temp_x.end(), temp_y.begin(), screen_mask.begin(), [&](const auto &i, const auto &j){
            return (i > 0 && i < SCREEN_WIDTH && j > 0 && j < SCREEN_HEIGHT);
        });
        QVector<qreal> new_X, new_Y;
        for (qsizetype i = 0; i < screen_mask.size(); i++){
            if (screen_mask[i]){
                new_X.push_back(X[i]);
                new_Y.push_back(Y[i]);
            }
        }
        X = new_X;
        Y = new_Y;
    }
    auto [XY_mask, W1, W2, W3] = is_inside_triangle(X, Y, x1 - x_min, y1 - y_min, x2 - x_min, y2 - y_min, x3 - x_min, y3 - y_min);

    {
        QVector<qreal> new_X, new_Y, new_W1, new_W2, new_W3;
        for (qsizetype i = 0; i < XY_mask.size(); i++){
            if (XY_mask[i]){
                new_X.push_back(X[i]);
                new_Y.push_back(Y[i]);
                new_W1.push_back(W1[i]);
                new_W2.push_back(W2[i]);
                new_W3.push_back(W3[i]);
            }
        }
        X = new_X;
        Y = new_Y;
        W1 = new_W1;
        W2 = new_W2;
        W3 = new_W3;
    }
    QVector<QVector<qreal>> W = {W1, W2, W3};
    return std::make_tuple(vector_sum(X, (qreal)x_min), vector_sum(Y, (qreal)y_min), W);
}

QVector<bool> MainWindow::screen_space(const QImage &draw_buffer, const QVector<qreal> &x, const QVector<qreal> &y){
    QVector<bool> result(x.size());
    std::transform(std::execution::unseq, x.begin(), x.end(), y.begin(),  result.begin(), [&](qreal i, qreal j){
        return (i >= 0 && i < draw_buffer.width() && j >= 0 && j < draw_buffer.height());
    });
    return result;
}

void MainWindow::draw_wireframe(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QColor &color, bool force_z){
    for (qsizetype i = 0; i < triangle.size(); i++) {
        auto x1 = triangle[i].second[0];
        auto y1 = triangle[i].second[1];
        auto z1 = triangle[i].second[2];
        auto x2 = triangle[(i + 1) % triangle.size()].second[0];
        auto y2 = triangle[(i + 1) % triangle.size()].second[1];
        auto z2 = triangle[(i + 1) % triangle.size()].second[2];
        auto l = (int)(qSqrt(qPow(x1 - x2, 2) + qPow(y1 - y2, 2)));
        auto temp_x = linspace(x1, x2, l);
        auto temp_y = linspace(y1, y2, l);
        auto temp_z = linspace(z1, z2, l);

        auto screen_mask = screen_space(draw_buffer, temp_x, temp_y);

        QVector<int> x, y, z;

        for (qsizetype j = 0; j < screen_mask.size(); j++) {
            if (screen_mask[j]){
                x.push_back((int)temp_x[j]);
                y.push_back((int)temp_y[j]);
                if (!force_z)
                    z.push_back((int)temp_z[j]);
            }
        }

        if (force_z)
            z = QVector<int>(x.size(), 0);

        for (qsizetype j = 0; j < x.size(); j++) {
            if (z_buffer[x[j]][y[j]] > z[j]){
                draw_buffer.setPixelColor(x[j], y[j], color);
                z_buffer[x[j]][y[j]] = z[j];
            }
        }
    }
}

void MainWindow::draw_lumbert(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QColor &color){
    auto [X, Y, W] = get_triangle_points(triangle[0].second[0], triangle[0].second[1], triangle[1].second[0], triangle[1].second[1], triangle[2].second[0], triangle[2].second[1]);
    if (X.size() > 0){
        auto Z = vector_sum(vector_multiplication(W[0], triangle[0].second[2]), vector_sum(vector_multiplication(W[1], triangle[1].second[2]), vector_multiplication(W[2], triangle[2].second[2])));

        for (qsizetype j = 0; j < X.size(); j++) {
            if (z_buffer[X[j]][Y[j]] > Z[j]){
                draw_buffer.setPixelColor(X[j], Y[j], color);
                z_buffer[X[j]][Y[j]] = Z[j];
            }
        }
    }
}

void MainWindow::draw_gouraud(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QVector<QColor> &colors){
    auto [X, Y, W] = get_triangle_points(triangle[0].second[0], triangle[0].second[1], triangle[1].second[0], triangle[1].second[1], triangle[2].second[0], triangle[2].second[1]);
    if (X.size() > 0){
        auto Z = vector_sum(vector_multiplication(W[0], triangle[0].second[2]), vector_sum(vector_multiplication(W[1], triangle[1].second[2]), vector_multiplication(W[2], triangle[2].second[2])));
        QVector<QColor> C;
        {
            QVector<QColor> C1, C2, C3;
            for (auto &i: W[0]) {
                C1.push_back(color_multiplication(colors[triangle[0].first], i));
            }
            for (auto &i: W[1]) {
                C2.push_back(color_multiplication(colors[triangle[1].first], i));
            }
            for (auto &i: W[2]) {
                C3.push_back(color_multiplication(colors[triangle[2].first], i));
            }
            for (qsizetype i = 0; i < C1.size(); i++) {
                C.push_back(color_sum(C1[i], color_sum(C2[i], C3[i])));
            }
        }
        for (qsizetype j = 0; j < X.size(); j++) {
            if (z_buffer[X[j]][Y[j]] > Z[j]){
                draw_buffer.setPixelColor(X[j], Y[j], C[j]);
                z_buffer[X[j]][Y[j]] = Z[j];
            }
        }
    }
}

void MainWindow::draw_phong(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QVector<qreal> &plane, const QColor &color, const QVector<PointLight> &lights){
    auto [X, Y, W] = get_triangle_points(triangle[0].second[0], triangle[0].second[1], triangle[1].second[0], triangle[1].second[1], triangle[2].second[0], triangle[2].second[1]);
    if (X.size() > 0){
        auto Z = vector_sum(vector_multiplication(W[0], triangle[0].second[2]), vector_sum(vector_multiplication(W[1], triangle[1].second[2]), vector_multiplication(W[2], triangle[2].second[2])));

        auto xs = X;
        auto ys = Y;

        auto factor = vector_division(FOCAL_LENGTH , Z);
        X = vector_division(vector_minus(X, (qreal)SCREEN_WIDTH / 2), factor);
        Y = vector_division(vector_minus(Y, (qreal)SCREEN_HEIGHT / 2), factor);
        const auto AMBIENT_COEFF = 0.05;
        const auto DIFFUSE_COEFF = 1;
        const auto SPECULAR_COEFF = 0;
        QVector<QColor> diffuse(X.size(), QColor(0, 0, 0)), specular(X.size(), QColor(0, 0, 0));
        for (auto &light: lights){
            const auto &[light_pos, light_color] = light.to_arr();
            auto xyz_transpose = matrix_transpose(QVector<QVector<qreal>>{X, Y, Z});
            auto l = vector_minus_mat(light_pos, xyz_transpose);

            QVector<qreal> n = {plane[0], plane[1], plane[2]};
            auto n_norm = qSqrt(qPow(n[0], 2) + qPow(n[1], 2) + qPow(n[2], 2));
            n = vector_division(n, n_norm);

            {
                auto temp_l = l;
                std::for_each(std::execution::unseq, temp_l.begin(), temp_l.end(), [&](auto &i){
                    std::for_each(std::execution::unseq, i.begin(), i.end(), [](auto &j){
                        j *= j;
                    });
                });
                QVector<qreal> l_norm(l.size());
                std::transform(std::execution::unseq, temp_l.begin(), temp_l.end(), l_norm.begin(), [&](const auto &i){
                    return qSqrt(std::accumulate(i.begin(), i.end(), (qreal)0.0));
                });
                std::transform(std::execution::unseq, l.begin(), l.end(), l_norm.begin(), l.begin(), [&](auto i, const auto &norm){
                    i[0] /= norm;
                    i[1] /= norm;
                    i[2] /= norm;
                    return i;
                });
            }

            auto r = vector_minus_mat(vector_multiplication(n, 2.0), l);
            {
                auto temp_r = r;
                std::for_each(std::execution::unseq, temp_r.begin(), temp_r.end(), [&](auto &i){
                    std::for_each(std::execution::unseq, i.begin(), i.end(), [](auto &j){
                        j *= j;
                    });
                });
                QVector<qreal> r_norm(r.size());
                std::transform(std::execution::unseq, temp_r.begin(), temp_r.end(), r_norm.begin(), [&](const auto &i){
                    return qSqrt(std::accumulate(i.begin(), i.end(), (qreal)0.0));
                });
                std::transform(std::execution::unseq, r.begin(), r.end(), r_norm.begin(), r.begin(), [&](auto i, const auto &norm){
                    i[0] /= norm;
                    i[1] /= norm;
                    i[2] /= norm;
                    return i;
                });
            }

            QVector<QColor> d(l.size());
            auto light_color_copy = light_color;
            auto t = dot(n, matrix_transpose(l));
            std::transform(std::execution::unseq, d.begin(), d.end(), t.begin(), d.begin(), [&](const auto &i, const auto &j){
                Q_UNUSED(i);
                return color_multiplication(color_multiplication(light_color_copy, j), DIFFUSE_COEFF);
            });
            std::transform(std::execution::unseq, d.begin(), d.end(), diffuse.begin(), diffuse.begin(), [&](const auto &i, const auto &j){
                return color_sum(i, j);
            });

            if (SPECULAR_COEFF == 0)
                continue;

            //if (std::max_element(d.begin(), d.end())->blackF() < 0.001 && std::max_element(d.begin(), d.end())->blueF() < 0.001 && std::max_element(d.begin(), d.end())->greenF() < .001)
            //    continue;

            auto neg_i = xyz_transpose;
            {
                std::for_each(std::execution::unseq, neg_i.begin(), neg_i.end(), [&](auto &i){
                    std::for_each(std::execution::unseq, i.begin(), i.end(), [](auto &j){
                        j = -j;
                    });
                });
                auto temp_neg_i = neg_i;
                std::for_each(std::execution::unseq, temp_neg_i.begin(), temp_neg_i.end(), [&](auto &i){
                    std::for_each(std::execution::unseq, i.begin(), i.end(), [](auto &j){
                        j *= j;
                    });
                });
                QVector<qreal> neg_i_norm(temp_neg_i.size());
                std::transform(std::execution::unseq, temp_neg_i.begin(), temp_neg_i.end(), neg_i_norm.begin(), [&](const auto &i){
                    return qSqrt(std::accumulate(i.begin(), i.end(), (qreal)0.0));
                });
                std::transform(std::execution::unseq, neg_i.begin(), neg_i.end(), neg_i_norm.begin(), neg_i.begin(), [&](auto i, const auto &norm){
                    i[0] /= norm;
                    i[1] /= norm;
                    i[2] /= norm;
                    return i;
                });
            }

            auto temp = matrix_multiplication(r, neg_i);
            QVector<qreal> temp_norm(temp.size());
            std::transform(std::execution::unseq, temp.begin(), temp.end(), temp_norm.begin(), [&](const auto &i){
                auto res = std::accumulate(i.begin(), i.end(), (qreal)0.0);
                res = res < 0 ? 0 : res;
                return qPow(res, 50);
            });
            QVector<QColor> temp2(temp_norm.size());
            std::transform(std::execution::unseq, temp_norm.begin(), temp_norm.end(), temp2.begin(), [&](const auto &i){
                return color_multiplication(color_multiplication(light_color_copy, i), SPECULAR_COEFF);
            });
            std::transform(std::execution::unseq, temp2.begin(), temp2.end(), specular.begin(), specular.begin(), [&](const auto &i, const auto &j){
                return color_sum(i, j);
            });
        }

        QVector<QColor> c(X.size(), QColor(0, 0, 0));
        std::transform(std::execution::unseq, diffuse.begin(), diffuse.end(), specular.begin(), c.begin(), [&](const auto &i, const auto &j){
            return color_multiplication(color_sum(color_sum(i, AMBIENT_COEFF), j), color);
        });

        for (qsizetype j = 0; j < X.size(); j++) {
            if (z_buffer[(qsizetype)xs[j]][(qsizetype)ys[j]] > Z[j]){
                draw_buffer.setPixelColor(xs[j], ys[j], c[j]);
                z_buffer[(qsizetype)xs[j]][(qsizetype)ys[j]] = Z[j];
            }
        }
    }
}


void MainWindow::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    auto t1 = std::chrono::high_resolution_clock::now();
    draw_buffer = clear_draw_buffer;
    z_buffer = clear_z_buffer;
    for (auto &light: point_lights) {
        auto cube = Cube(0.1);
        cube.translate_cube(light.point.x, light.point.y, light.point.z);
        auto triangles = cube.draw_cube(camera_pos, camera_angles, {});
        for (auto &polygon: triangles){
            draw_lumbert(draw_buffer, z_buffer, polygon.second, light.color);
        }
    }
    for (auto &obj: objects){
        auto triangles = obj.draw_cube(camera_pos, camera_angles, point_lights, current_shading);
        auto [_, normals] = obj.get_c_and_norm(obj.transform_vertices(camera_pos, camera_angles, false));
        for (auto &polygon: triangles){
            auto &triangle = polygon.second;
            auto &face_ind = polygon.first;
            switch (current_shading) {
            case WIREFRAME:
                draw_wireframe(draw_buffer, z_buffer, triangle, obj.color);
                break;

            case LAMBERT:
                draw_lumbert(draw_buffer, z_buffer, triangle, obj.lambert_colors[face_ind]);
                break;

            case GOURAUD:
                draw_gouraud(draw_buffer, z_buffer, triangle, obj.gouraud_colors);
                break;

            case PHONG:
            {
                auto lights = point_lights;
                for (auto &i: lights) {
                    i.point.x += camera_pos[0];
                    i.point.y += camera_pos[1];
                    i.point.z += camera_pos[2];
                    i.point = i.point.rotate_yx(camera_angles[0], camera_angles[1]);
                }
                draw_phong(draw_buffer, z_buffer, triangle, normals[face_ind], obj.color, lights);
            }
                break;

            default:
                break;
            }
        }
    }
    QPainter painter(this);
    painter.drawImage(0, 0, draw_buffer);
    auto t2 = std::chrono::high_resolution_clock::now();
    qDebug() << duration_cast<std::chrono::duration<double>>(t2 - t1).count();
}

void MainWindow::keyPressEvent(QKeyEvent *event){
    switch (event->key()) {
    case Qt::Key_E:
        camera_pos[1] += 0.1;
        break;

    case Qt::Key_Q:
        camera_pos[1] -= 0.1;
        break;

    case Qt::Key_A:
        camera_pos[0] += 0.1;
        break;

    case Qt::Key_D:
        camera_pos[0] -= 0.1;
        break;

    case Qt::Key_S:
        camera_pos[2] += 0.1;
        break;

    case Qt::Key_W:
        camera_pos[2] -= 0.1;
        break;

    case Qt::Key_Up:
        camera_angles[0] += 0.1;
        break;

    case Qt::Key_Down:
        camera_angles[0] -= 0.1;
        break;

    case Qt::Key_Left:
        camera_angles[1] += 0.1;
        break;

    case Qt::Key_Right:
        camera_angles[1] -= 0.1;
        break;

    case Qt::Key_1:
        current_shading = WIREFRAME;
        break;

    case Qt::Key_2:
        current_shading = LAMBERT;
        break;

    case Qt::Key_3:
        current_shading = GOURAUD;
        break;

    case Qt::Key_4:
        current_shading = PHONG;
        break;

    case Qt::Key_T:
        if (selected_object != NULL){
            selected_object->angles[0] -= 0.4;
        }
        break;

    case Qt::Key_G:
        if (selected_object != NULL){
            selected_object->angles[0] += 0.4;
        }
        break;

    case Qt::Key_F:
        if (selected_object != NULL){
            selected_object->angles[1] -= 0.4;
        }
        break;

    case Qt::Key_H:
        if (selected_object != NULL){
            selected_object->angles[1] += 0.4;
        }
        break;

    default:
        break;
    }
    repaint();
}
