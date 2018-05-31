using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using ToyPhysics.Collision;

namespace ToyPhysics.Tests.Collision
{
	[TestClass]
	public class DynamicTreeTest
	{
		[TestMethod]
		public void TestManipulation()
		{
			var tree = new DynamicTree();

			var aabb1 = new AABB( new Vector2D( 21.9995f, 1.4995f ), new Vector2D( 28.0005f, 2.2005f ) );
			var proxy1 = tree.AddProxy( aabb1, null );

			Assert.AreEqual( 1, tree.Count );
			Assert.AreEqual( 0, proxy1.Depth );
			Assert.IsTrue( proxy1.IsLeaf );
			Assert.IsTrue( proxy1.IsRoot );

			var aabb2 = new AABB( new Vector2D( 21.9995f, 1.9995f ), new Vector2D( 22.5005f, 3.7005f ) );
			var proxy2 = tree.AddProxy( aabb2, null );

			Assert.AreEqual( 3, tree.Count );
			Assert.AreEqual( 0, proxy2.Depth );
			Assert.IsTrue( proxy2.IsLeaf );
			Assert.IsFalse( proxy2.IsRoot );

			var aabb3 = new AABB( new Vector2D( 27.4995f, 4.9995f ), new Vector2D( 28.0005f, 6.7005f ) );
			var proxy3 = tree.AddProxy( aabb3, null );

			Assert.AreEqual( 5, tree.Count );
			Assert.AreEqual( 0, proxy3.Depth );
			Assert.IsTrue( proxy3.IsLeaf );
			Assert.IsFalse( proxy3.IsRoot );

			var aabb4 = new AABB( new Vector2D( 23.4495f, 1.1495f ), new Vector2D( 24.9505f, 3.8505f ) );
			var proxy4 = tree.AddProxy( aabb4, null );

			Assert.AreEqual( 7, tree.Count );
			Assert.AreEqual( 0, proxy4.Depth );
			Assert.IsTrue( proxy4.IsLeaf );
			Assert.IsFalse( proxy4.IsRoot );

			tree.RemoveProxy( proxy2 );

			Assert.AreEqual( 5, tree.Count );
			Assert.AreEqual( 2, tree.FreeCount );
			Assert.AreEqual( -1, proxy2.Depth );
			Assert.IsTrue( proxy2.IsFree );

			aabb4 = new AABB( new Vector2D( 23.8495f, 5.9495f ), new Vector2D( 25.3505f, 8.6505f ) );
			tree.MoveProxy( proxy4, aabb4, new Vector2D( 1, 1 ) );

			Assert.AreEqual( 5, tree.Count );
			Assert.AreEqual( 2, tree.FreeCount );
		}

		[TestMethod]
		public void TestOverlap()
		{
			var tree = CreateTestTree();

			var aabb6 = new AABB( new Vector2D( 20.4400f, 2.6600f ), new Vector2D( 26.3600f, 8.9400f ) );

			var hits = new List<DynamicTreeNode>();
			tree.Overlap( aabb6, node =>
			{
				hits.Add( node );
				return true;
			} );

			Assert.AreEqual( 3, hits.Count );
			Assert.IsTrue( hits.TrueForAll( h => (int)h.Context == 2 || (int)h.Context == 4 || (int)h.Context == 5 ) );
		}

		[TestMethod]
		public void TestRaycast()
		{
			var tree = CreateTestTree();

			var input = new DynamicTree.RayCastInput()
			{
				Point1 = new Vector2D( 27.4995f, 6.9995f ),
				Point2 = new Vector2D( 28.0005f, 8.2005f ),
				MaxFraction = 1,
			};

			var hits = new List<DynamicTreeNode>();
			tree.RayCast( input, (i, node) =>
			{
				hits.Add( node );
				return 1;
			} );

			Assert.AreEqual( 1, hits.Count );
			Assert.IsTrue( hits.TrueForAll( h => (int)h.Context == 5 ) );
		}

		private static DynamicTree CreateTestTree()
		{
			var tree = new DynamicTree();

			var aabb1 = new AABB( new Vector2D( 21.9995f, 1.4995f ), new Vector2D( 28.0005f, 2.2005f ) );
			var proxy1 = tree.AddProxy( aabb1, 1 );

			var aabb2 = new AABB( new Vector2D( 21.9995f, 1.9995f ), new Vector2D( 22.5005f, 3.7005f ) );
			var proxy2 = tree.AddProxy( aabb2, 2 );

			var aabb3 = new AABB( new Vector2D( 27.4995f, 4.9995f ), new Vector2D( 28.0005f, 6.7005f ) );
			var proxy3 = tree.AddProxy( aabb3, 3 );

			var aabb4 = new AABB( new Vector2D( 23.8495f, 5.9495f ), new Vector2D( 25.3505f, 8.6505f ) );
			var proxy4 = tree.AddProxy( aabb4, 4 );

			var aabb5 = new AABB( new Vector2D( 25.4995f, 6.4995f ), new Vector2D( 28.0005f, 7.2005f ) );
			var proxy5 = tree.AddProxy( aabb5, 5 );
			return tree;
		}

	}
}
