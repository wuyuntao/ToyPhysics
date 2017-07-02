namespace ToyPhysics.Collision
{
    public sealed class ContactPoint
    {
        public Vector2D LocalPoint;
    }

    public sealed class ContactManifold
    {
        public Vector2D LocalPoint;
        public Vector2D LocalNormal;
        public ContactPoint[] Points;

    }
}
