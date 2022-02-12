#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QtMath>
#include <QVector>
#include <QPair>
#include <QColor>
#include <QImage>
#include <QPainter>

#include <algorithm>
#include <tuple>
#include <limits>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

const qreal FOV = qDegreesToRadians(90);
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const qreal FOCAL_LENGTH = SCREEN_WIDTH / 2 * qCos(FOV / 2) / qSin(FOV / 2);

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent *event);
    void keyPressEvent(QKeyEvent *);

private:
    Ui::MainWindow *ui;

    enum Mode{
        MOVING,
        ROTATING
    };
    enum Direction{
        UP,
        DOWN,
        RIGHT,
        LEFT,
        FORWARD,
        BACKWARDS
    };
    enum Shading{
        WIREFRAME,
        LAMBERT,
        GOURAUD,
        PHONG
    };

    static QVector<QVector<qreal>> rotation_matrix(QVector<qreal> axis, qreal theta);
    static qreal dot(const QVector<qreal> &a, const QVector<qreal> &b);
    static QVector<qreal> dot(const QVector<QVector<qreal>> &matrix, const QVector<qreal> &vector);
    static QVector<qreal> dot(const QVector<qreal> &vector, const QVector<QVector<qreal>> &matrix);

    struct Point3D{
        qreal x, y, z;
        Point3D(const qreal x, const qreal y, const qreal z);
        Point3D operator-(const Point3D &other) const;
        QVector<qreal> to_arr() const;
        QPair<qreal, qreal> determ_sincos(qreal angle) const;
        Point3D rotate_X(qreal angle) const;
        Point3D rotate_Y(qreal angle) const;
        Point3D rotate_Z(qreal angle) const;
        Point3D rotate(qreal a_x, qreal a_y, qreal a_z) const;
        Point3D rotate_yx(qreal a_x, qreal a_y) const;
        Point3D rotate_xy(qreal a_x, qreal a_y) const;
        Point3D project() const;
    };

    struct PointLight{
        QColor color;
        Point3D point;
        PointLight(const Point3D &point, const QColor &color);
        void translate_cube(const qreal x, const qreal y, const qreal z);
        QPair<QVector<qreal>, QColor> to_arr() const;
    };

    template <class T>
    static QVector<QVector<T>> vector_minus_mat(const QVector<T> &a, const QVector<QVector<T>> &b);
    static QVector<QVector<qreal>> vector_minus_mat(const QVector<qreal> &a, const QVector<Point3D> &b);
    static QVector<qreal> crossProduct(const QVector<qreal> &a, const QVector<qreal> &b);
    static Point3D crossProduct(const Point3D &a, const Point3D &b);
    static qreal linalg_norm(const QVector<qreal> &x);
    static QVector<qreal> linspace(qreal x1, qreal x2, int num);
    static QColor color_multiplication(const QColor &a, const QColor &b);
    static QColor color_multiplication(const QColor &a, const qreal &b);
    static QColor color_sum(const QColor &a, const QColor &b);
    template <class T>
    static QVector<QVector<T>> matrix_transpose(const QVector<QVector<T>> &source);
    template <class T>
    static QVector<QVector<T>> matrix_multiplication(const QVector<QVector<T>> &a, const QVector<QVector<T>> &b);

    template <class T>
    static QVector<T> vector_division(QVector<T> a, const T &b);
    template <class T>
    static QVector<T> vector_division(const T &b, QVector<T> a);
    template <class T>
    static QVector<T> vector_division(const QVector<T> &a, const QVector<T> &b);
    template <class T>
    static QVector<T> vector_sum(const QVector<T> &a, const T &b);
    template <class T>
    static QVector<T> vector_sum(const QVector<T> &a, const QVector<T> &b);
    template <class T>
    static QVector<T> vector_multiplication(const QVector<T> &a, const T &b);
    template <class T>
    static QVector<T> vector_multiplication(const QVector<T> &a, const QVector<T> &b);
    template <class T>
    static QVector<T> vector_minus(const QVector<T> &a, const T &b);
    template <class T>
    static QVector<T> vector_minus(const T &b, const QVector<T> &a);
    template <class T>
    static QVector<T> vector_minus(const QVector<T> &a, const QVector<T> &b);
    static QVector<qreal> vector_abs(const QVector<qreal> &a);

    static QPair<QVector<QVector<qreal>>, QVector<QVector<qreal>>> get_c_and_norm(const QVector<QVector<int>> &faces, const QVector<Point3D> &vertices, bool return_planes = false);
    static QVector<QColor> lambert_lighting(const QVector<QVector<int>> &faces, const QVector<Point3D> &vertices, const QVector<PointLight> &point_lights, const QColor &color);
    static QVector<QColor> gouraud_lighting(const QVector<QVector<int>> &faces, const QVector<Point3D> &vertices, const QVector<QVector<int>> &vertices_faces, const QVector<PointLight> &point_lights, const QColor &color);

    struct Cube{
        QVector<Point3D> vertices;
        QVector<QVector<int>> faces, vertices_faces;
        QVector<qreal> angles, pos;
        QVector<QColor> lambert_colors, gouraud_colors;
        QColor color;
        Cube(qreal size = 1, QColor color = Qt::black);
        QVector<Point3D> transform_vertices(const QVector<qreal> &camera_pos, const QVector<qreal> &camera_angles, bool project=true) const;
        QVector<QPair<qsizetype, qreal>> calculate_avg_z(const QVector<Point3D> &vertices) const;
        QPair<QVector<QVector<qreal>>, QVector<QVector<qreal>>> get_c_and_norm(const QVector<Point3D> &transformed_vertices, bool return_d=false) const;
        void translate_cube(const qreal x, const qreal y, const qreal z);
        void rotate_cube(Direction direction, qreal speed = 1);
        void lambert_make_color(const QVector<PointLight> &point_lights);
        void gouraud_make_color(const QVector<PointLight> &point_lights);
        QVector<QPair<int, QVector<QPair<int, QVector<qreal>>>>> draw_cube(const QVector<qreal> &camera_pos, const QVector<qreal> &camera_angles, const QVector<PointLight> &point_lights, Shading shading = WIREFRAME, bool triangles = true);
    };

    static qreal area(qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3);
    static QVector<qreal> area(const QVector<qreal> &x1, const QVector<qreal> &y1, qreal x2, qreal y2, qreal x3, qreal y3);
    static QVector<qreal> area(qreal x1, qreal y1, const QVector<qreal> &x2, const QVector<qreal> &y2, qreal x3, qreal y3);
    static QVector<qreal> area(qreal x1, qreal y1, qreal x2, qreal y2, const QVector<qreal> &x3, const QVector<qreal> &y3);
    static std::tuple<QVector<bool>, QVector<qreal>, QVector<qreal>, QVector<qreal>> is_inside_triangle(const QVector<qreal> &x, const QVector<qreal> &y, qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3);
    static std::tuple<QVector<qreal>, QVector<qreal>, QVector<QVector<qreal>>> get_triangle_points(qreal x1, qreal y1, qreal x2, qreal y2, qreal x3, qreal y3);
    static QVector<bool> screen_space(const QImage &draw_buffer, const QVector<qreal> &x, const QVector<qreal> &y);
    static void draw_wireframe(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QColor &color, bool force_z = false);
    static void draw_lumbert(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QColor &color);
    static void draw_gouraud(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QVector<QColor> &colors);
    static void draw_phong(QImage &draw_buffer, QVector<QVector<qreal>> &z_buffer, const QVector<QPair<int, QVector<qreal>>> &triangle, const QVector<qreal> &plane, const QColor &color, const QVector<PointLight> &lights);

    QImage draw_buffer, clear_draw_buffer;
    Shading current_shading;
    QVector<Cube> objects;
    QVector<qreal> camera_pos, camera_angles;
    Cube *selected_object = NULL;
    QVector<PointLight> point_lights;
    QVector<QVector<qreal>> z_buffer, clear_z_buffer;
};
#endif // MAINWINDOW_H
