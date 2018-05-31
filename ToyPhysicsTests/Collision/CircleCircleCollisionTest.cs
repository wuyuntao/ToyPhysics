using Microsoft.VisualStudio.TestTools.UnitTesting;
using ToyPhysics.Collision;
using ToyPhysics.Shapes;

namespace ToyPhysics.Tests.Collision
{
	[TestClass]
	public class CircleCircleCollisionTest
	{
		[TestMethod]
		public void TestCircleCircleCollision()
		{
			var c1 = new CircleShape( new Vector2D( 0.1f, 0.2f ), 1 );
			var t1 = new Transform2D( new Vector2D( 0, 0 ), new Rotation2D( 0 ) );

			var c2 = new CircleShape( new Vector2D( -0.1f, -0.2f ), 1 );
			var t2 = new Transform2D( new Vector2D( 0.5f, 0 ), new Rotation2D( 0 ) );

			var r = CollisionT.Collide( out ContactManifold manifold, c1, t1, c2, t2 );
			Assert.IsTrue( r );
		}
	}
}
