namespace ToyPhysics.Shapes
{
    public abstract class Shape
    {
        private Vector2D center;

        protected Shape(Vector2D center)
        {
            this.center = center;
        }

        public Vector2D Center => center;
    }
}
