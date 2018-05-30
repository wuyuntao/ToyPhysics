namespace ToyPhysics.Collision
{
    public struct AABB
    {
        public Vector2D LowerBound, UpperBound;

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

        public Vector2D Center
        {
            get { return 0.5f * (LowerBound + UpperBound); }
        }

        public Vector2D Extents
        {
            get { return 0.5f * (UpperBound - LowerBound); }
        }

        public void Combine(AABB aabb1, AABB aabb2)
        {
            LowerBound = Vector2D.Min(aabb1.LowerBound, aabb2.LowerBound);
            UpperBound = Vector2D.Max(aabb1.UpperBound, aabb2.UpperBound);
        }

        public bool Contains(AABB aabb)
        {
            return LowerBound.X <= aabb.LowerBound.X &&
                LowerBound.Y <= aabb.LowerBound.Y &&
                aabb.UpperBound.X <= UpperBound.X &&
                aabb.UpperBound.Y <= UpperBound.Y;
        }
    }
}
