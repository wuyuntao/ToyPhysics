namespace ToyPhysics
{
    static class Configuration
    {
        // A small length used as a collision and constraint tolerance. Usually it is
        // chosen to be numerically significant, but visually insignificant
        public const float LinearSlop = 0.005f;

        // The radius of the polygon/edge shape skin. This should not be modified. Making
        // this smaller means polygons will have an insufficient buffer for continuous collision.
        // Making it larger may create artifacts for vertex collision.
        public const float PolygonRadius = 2f * LinearSlop;

        public const float FatAABBExtension = 0.1f;

        public const float AABBDisplacementMultiplier = 1.1f;
    }
}
