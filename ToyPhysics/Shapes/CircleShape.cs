namespace ToyPhysics.Shapes
{
    public class CircleShape : Shape
    {
        private float radius;

        public CircleShape(Vector2D center, float radius)
            : base(center)
        {
            this.radius = radius;
        }

        public float Radius => radius;
    }
}
