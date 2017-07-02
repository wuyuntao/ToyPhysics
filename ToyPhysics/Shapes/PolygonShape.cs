using System;
using System.Diagnostics;

namespace ToyPhysics.Shapes
{
    public class PolygonShape : Shape
    {
        private Vector2D[] vertices;
        private Vector2D[] normals;

        public PolygonShape(Vector2D center, Vector2D[] vertices)
            : base(center)
        {
            SetVertices(vertices);
        }

        private void SetVertices(Vector2D[] vertices)
        {
#if DEBUG
            if (vertices == null)
                throw new ArgumentNullException(nameof(vertices));

            if (vertices.Length < 3)
                throw new ArgumentOutOfRangeException(nameof(vertices));

            // 验证定点是否顺时针方向
            // 验证定点是否为凸包
#endif
            this.vertices = vertices;

            // 计算法线，并验证不存在长度为0的边
            normals = new Vector2D[vertices.Length];
            for (var i = 0; i < vertices.Length; ++i)
            {
                int inext = i + 1 < vertices.Length ? i + 1 : 0;
                var edge = vertices[inext] - vertices[i];
                Debug.Assert(edge.LengthSquared() > float.Epsilon * float.Epsilon);
                var normal = Vector2D.Cross(edge, 1.0f);
                normal.Normalize();
                normals[i] = normal;
            }
        }

        public Vector2D[] Verticles => vertices;

        public Vector2D[] Normals => normals;
    }
}
