using System;
using System.Diagnostics;
using ToyPhysics.Shapes;

namespace ToyPhysics.Collision
{
    static class CollisionT
    {
        public static bool Collide(out ContactManifold manifold,
            CircleShape circle1, Transform2D transform1,
            CircleShape circle2, Transform2D transform2)
        {
            var center1 = transform1.Transform(circle1.Center);
            var center2 = transform2.Transform(circle2.Center);

            // 两个圆是否碰撞，计算两个圆心的距离，是否比两者的半径之和小
            var totalRadius = circle1.Radius + circle2.Radius;
            if ((center1 - center2).LengthSquared() > totalRadius * totalRadius)
            {
                manifold = null;
                return false;
            }

            manifold = new ContactManifold()
            {
                LocalPoint = circle1.Center,
                LocalNormal = Vector2D.Zero,        // 为什么圆和圆的法线为 0 向量？
                Points = new[] { new ContactPoint() { LocalPoint = circle2.Center }
                },
            };
            return true;
        }

        public static bool Collide(out ContactManifold manifold,
            PolygonShape polygon1, Transform2D transform1,
            CircleShape circle2, Transform2D transform2)
        {
            // 将圆心坐标转换到多边形的本地坐标系中
            var c = transform2.InverseTransform(transform1.Transform(circle2.Center));

            var radius = Configuration.PolygonRadius + circle2.Radius;
            var normalIndex = 0;
            var seperation = float.MinValue;
            for (int i = 0; i < polygon1.Verticles.Length; ++i)
            {
                // 将圆心到顶点的向量，投影到顶点所在边的法线上
                var s = Vector2D.Dot(polygon1.Normals[i], c - polygon1.Verticles[i]);
                // 如果投影长度大于圆的半径，即不存在相交
                if (s > radius)
                {
                    manifold = null;
                    return false;
                }

                // 找到相交最浅的顶点
                if (s > seperation)
                {
                    normalIndex = i;
                    seperation = s;
                }
            }

            // 得到最浅相交的接触边的两个顶点
            var v1 = polygon1.Verticles[normalIndex];
            var v2 = polygon1.Verticles[normalIndex + 1 < polygon1.Verticles.Length ? normalIndex + 1 : 0];

            // 如果圆心在多边形内部
            if (seperation < float.Epsilon)
            {
                manifold = new ContactManifold()
                {
                    LocalPoint = (v1 + v2) * 0.5f,
                    LocalNormal = polygon1.Normals[normalIndex],
                    Points = new[] { new ContactPoint() { LocalPoint = circle2.Center } }
                };
                return true;
            }

            // 如果圆心的投影在 v1 顶点的外侧
            var u1 = Vector2D.Dot(c - v1, v2 - v1);
            if (u1 <= 0f)
            {
                if ((c - v1).LengthSquared() > radius * radius)
                {
                    manifold = null;
                    return false;
                }

                manifold = new ContactManifold()
                {
                    LocalPoint = v1,
                    LocalNormal = c - v1,
                    Points = new[] { new ContactPoint() { LocalPoint = circle2.Center } }
                };
                manifold.LocalNormal.Normalize();
                return true;
            }

            // 如果圆心的投影在 v2 顶点的外侧
            var u2 = Vector2D.Dot(c - v2, v1 - v2);
            if (u2 <= 0f)
            {
                if ((c - v2).LengthSquared() > radius * radius)
                {
                    manifold = null;
                    return false;
                }

                manifold = new ContactManifold()
                {
                    LocalPoint = v2,
                    LocalNormal = c - v2,
                    Points = new[] { new ContactPoint() { LocalPoint = circle2.Center } }
                };
                manifold.LocalNormal.Normalize();
                return true;
            }

            // 如果圆心的投影在 v1 和 v2 顶点的边上
            {
                var faceCenter = 0.5f * (v1 + v2);
                var s = Vector2D.Dot(c - faceCenter, polygon1.Normals[normalIndex]);
                if (s > radius)
                {
                    manifold = null;
                    return false;
                }

                manifold = new ContactManifold()
                {
                    LocalPoint = faceCenter,
                    LocalNormal = polygon1.Normals[normalIndex],
                    Points = new[] { new ContactPoint() { LocalPoint = circle2.Center } }
                };
                return true;
            }
        }

        public static bool Collide(out ContactManifold manifold,
            PolygonShape polygon1, Transform2D transform1,
            PolygonShape polygon2, Transform2D transform2)
        {
            var radius = Configuration.PolygonRadius * 2;

            var seperation1 = FindMaxSeperation(out int edge1, polygon1, transform1, polygon2, transform2);
            if (seperation1 > radius)
            {
                manifold = null;
                return false;
            }

            var seperation2 = FindMaxSeperation(out int edge2, polygon2, transform2, polygon1, transform1);
            if (seperation2 > radius)
            {
                manifold = null;
                return false;
            }

            var flip = 0;
            var tol = 0.1f * Configuration.LinearSlop;

            if (seperation2 > seperation1 + tol)
            {
                MathT.Swap(ref polygon1, ref polygon2);
                MathT.Swap(ref transform1, ref transform2);
                MathT.Swap(ref edge1, ref edge2);
                flip = 1;
            }

            var incidentEdge = FindIncidentEdge(polygon1, transform1, edge1, polygon2, transform2);

            throw new NotImplementedException();
        }

        private static float FindMaxSeperation(out int edgeIndex,
            PolygonShape polygon1, Transform2D transform1,
            PolygonShape polygon2, Transform2D transform2)
        {
            // 找到 polygon2 在 polygon1 的各个法线上投影最大的顶点
            // 如果投影 > 0 表示在该法线投影上，两个 polygon 没有相交
            // 根据分离轴的算法，即两个 polygon 没有相交
            var transform = transform1.Transform(transform2);

            edgeIndex = -1;
            var maxSeperation = float.MinValue;
            for (int i = 0; i < polygon1.Normals.Length; ++i)
            {
                // 将 polygon1 的本地坐标系转换到 polygon2 的本地坐标系
                var v1 = transform.Transform(polygon1.Verticles[i]);
                var n1 = transform.Transform(polygon1.Normals[i]);

                // 计算 polygon2 顶点在 normal 上投影最深的点
                var si = float.MaxValue;
                for (int j = 0; j < polygon2.Verticles.Length; ++j)
                {
                    var sij = Vector2D.Dot(polygon2.Verticles[j] - v1, n1);
                    if (sij < si)
                        si = sij;
                }

                if (si > maxSeperation)
                {
                    maxSeperation = si;
                    edgeIndex = i;
                }
            }

            return maxSeperation;
        }

        private static Vector2D[] FindIncidentEdge(
            PolygonShape polygon1, Transform2D transform1, int edge1,
            PolygonShape polygon2, Transform2D transform2)
        {
            Debug.Assert(0 <= edge1 && edge1 < polygon1.Normals.Length);

            // 将 polygon1 的本地坐标系转换到 polygon2 的本地坐标系
            var normal1 = transform2.Q.InverseRotate(transform1.Q.Rotate(polygon1.Normals[edge1]));

            // 找到和法线最垂直的边索引，即入射边
            var minS = float.MaxValue;
            var sIndex = 0;
            for (int i = 0; i < polygon2.Normals.Length; ++i)
            {
                var s = Vector2D.Dot(normal1, polygon2.Normals[i]);
                if (s < minS)
                {
                    minS = s;
                    sIndex = i;
                }
            }

            var sIndexNext = sIndex + 1 < polygon2.Verticles.Length ? sIndex + 1 : 0;
            var incidentEdge = new Vector2D[2];
            incidentEdge[0] = transform2.Transform(polygon1.Verticles[sIndex]);
            incidentEdge[1] = transform2.Transform(polygon1.Verticles[sIndexNext]);

            return incidentEdge;
        }
    }
}
