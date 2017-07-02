using ToyPhysics.Shapes;

namespace ToyPhysics.Collision
{
    static class Collision
    {
        public static bool Collide(out CollisionManifold manifold,
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

            manifold = new CollisionManifold()
            {
                LocalPoint = circle1.Center,
                LocalNormal = Vector2D.Zero,        // 为什么圆和圆的法线为 0 向量？
                Points = new[] { new CollisionManifold.ContactPoint() { LocalPoint = circle2.Center }
                },
            };
            return true;
        }

        public static bool Collide(out CollisionManifold manifold,
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
                manifold = new CollisionManifold()
                {
                    LocalPoint = (v1 + v2) * 0.5f,
                    LocalNormal = polygon1.Normals[normalIndex],
                    Points = new[] { new CollisionManifold.ContactPoint() { LocalPoint = circle2.Center } }
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

                manifold = new CollisionManifold()
                {
                    LocalPoint = v1,
                    LocalNormal = c - v1,
                    Points = new[] { new CollisionManifold.ContactPoint() { LocalPoint = circle2.Center } }
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

                manifold = new CollisionManifold()
                {
                    LocalPoint = v2,
                    LocalNormal = c - v2,
                    Points = new[] { new CollisionManifold.ContactPoint() { LocalPoint = circle2.Center } }
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

                manifold = new CollisionManifold()
                {
                    LocalPoint = faceCenter,
                    LocalNormal = polygon1.Normals[normalIndex],
                    Points = new[] { new CollisionManifold.ContactPoint() { LocalPoint = circle2.Center } }
                };
                return true;
            }
        }
    }
}
