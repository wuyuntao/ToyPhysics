namespace ToyPhysics.Collision
{
    public struct AABB
    {
        public Vector2D LowerBound, UpperBound;

        public AABB(Vector2D lowerBound, Vector2D upperBound)
        {
            LowerBound = lowerBound;
            UpperBound = upperBound;
        }

        public bool Contains(AABB aabb)
        {
            return LowerBound.X <= aabb.LowerBound.X &&
                LowerBound.Y <= aabb.LowerBound.Y &&
                aabb.UpperBound.X <= UpperBound.X &&
                aabb.UpperBound.Y <= UpperBound.Y;
        }

        public bool Overlap(AABB aabb)
        {
            var d1 = aabb.LowerBound - UpperBound;
            var d2 = LowerBound - aabb.UpperBound;

            if (d1.X > 0.0f || d1.Y > 0.0f)
                return false;

            if (d2.X > 0.0f || d2.Y > 0.0f)
                return false;

            return true;
        }

        public AABB Combine(AABB aabb2)
        {
            return new AABB(
                Vector2D.Min(LowerBound, aabb2.LowerBound),
                Vector2D.Max(UpperBound, aabb2.UpperBound));
        }

        public AABB Enlarge(Vector2D size)
        {
            return new AABB(LowerBound - size, UpperBound + size);
        }

        public Vector2D Center => 0.5f * (LowerBound + UpperBound);

        public Vector2D Extents => 0.5f * (UpperBound - LowerBound);

        public float Perimeter => 4 * (Extents.X + Extents.Y);

        public bool IsValid
        {
            get
            {
                return LowerBound.IsValid &&
                    UpperBound.IsValid &&
                    (UpperBound.X - LowerBound.X) >= 0f &&
                    (UpperBound.Y - LowerBound.Y) >= 0f;
            }
        }
    }
}
