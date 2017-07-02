using System;
using System.Runtime.InteropServices;

namespace ToyPhysics
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Rotation2D : IEquatable<Rotation2D>, IFormattable
    {
        public float S, C;

        public Rotation2D(float radian)
        {
            S = (float)Math.Sin(radian);
            C = (float)Math.Cos(radian);
        }

        public float Radian
        {
            get
            {
                return (float)Math.Atan2(S, C);
            }
            set
            {
                S = (float)Math.Sin(value);
                C = (float)Math.Cos(value);
            }
        }

        /// <summary>
        /// 对向量应用旋转
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Vector2D Rotate(Vector2D point)
        {
            // 类似于应用变换矩阵的旋转部分
            return new Vector2D(C * point.X - S * point.Y, S * point.X + C * point.Y);
        }

        /// <summary>
        /// 对向量应用逆向旋转
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public Vector2D InverseRotate(Vector2D point)
        {
            // 逆向旋转等价于做弧度值为 -Radian（即 Sin 值为 -S, Cos 值不变）的旋转
            return new Vector2D(C * point.X + S * point.Y, -S * point.X + C * point.Y);
        }

        /// <summary>
        /// 对旋转应用旋转
        /// </summary>
        /// <param name="rotation"></param>
        /// <returns></returns>
        public Rotation2D Rotate(Rotation2D rotation)
        {
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
            // s = qs * rc + qc * rs
            // c = qc * rc - qs * rs
            return new Rotation2D()
            {
                S = S * rotation.C + C * rotation.S,
                C = C * rotation.C - S * rotation.S,
            };
        }

        /// <summary>
        /// 对旋转应用逆向旋转
        /// </summary>
        /// <param name="rotation"></param>
        /// <returns></returns>
        public Rotation2D InverseRotate(Rotation2D rotation)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
            // s = qc * rs - qs * rc
            // c = qc * rc + qs * rs
            return new Rotation2D()
            {
                S = C * rotation.S - S * rotation.C,
                C = C * rotation.C + S * rotation.S,
            };
        }

        #region IEquatable

        public override bool Equals(object obj)
        {
            if (!(obj is Rotation2D))
                return false;

            return Equals((Rotation2D)obj);
        }

        public bool Equals(Rotation2D obj)
        {
            return obj.S == S && obj.C == C;
        }

        public override int GetHashCode()
        {
            return (S.GetHashCode() * 0x18d ^ C.GetHashCode()).GetHashCode();
        }

        #endregion

        #region IFormattable

        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format(Radian.ToString(format, formatProvider));
        }

        public string ToString(string format)
        {
            return string.Format(Radian.ToString(format));
        }

        public string ToString(IFormatProvider formatProvider)
        {
            return string.Format(Radian.ToString(formatProvider));
        }

        public override string ToString()
        {
            return string.Format(Radian.ToString());
        }

        #endregion
    }
}
