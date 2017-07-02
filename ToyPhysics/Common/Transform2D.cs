using System;
using System.Runtime.InteropServices;

namespace ToyPhysics
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Transform2D
    {
        public Vector2D P;
        public Rotation2D Q;

        public Transform2D(ref Vector2D position, ref Rotation2D rotation)
        {
            P = position;
            Q = rotation;
        }

        /// <summary>
        /// 将本地坐标系变换到世界坐标系
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Vector2D Transform(Vector2D point)
        {
            return P + Q.Rotate(point);
        }

        /// <summary>
        /// 将世界坐标系逆变换到本地坐标系
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Vector2D InverseTransform(Vector2D point)
        {
            return Q.InverseRotate(point - P);
        }

        public Transform2D Transform(Transform2D transform)
        {
            return new Transform2D()
            {
                P = P + Q.Rotate(transform.P),
                Q = Q.Rotate(transform.Q)
            };
        }

        public Transform2D InverseTransform(Transform2D transform)
        {
            return new Transform2D()
            {
                P = Q.InverseRotate(transform.P - P),
                Q = Q.InverseRotate(transform.Q),
            };
        }
    }
}