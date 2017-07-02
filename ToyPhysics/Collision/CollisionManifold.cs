namespace ToyPhysics.Collision
{
    public sealed class CollisionManifold
    {
        public Vector2D LocalPoint;
        public Vector2D LocalNormal;
        public ContactPoint[] Points;

        public sealed class ContactPoint
        {
            public Vector2D LocalPoint;
        }
    }
}
