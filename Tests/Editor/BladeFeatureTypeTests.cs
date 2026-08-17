using NUnit.Framework;

namespace BladeContact.Tests
{
    public sealed class BladeFeatureTypeTests
    {
        [Test]
        public void SharpEdge_IsDistinctFromBluntEdge()
        {
            Assert.AreNotEqual(BladeFeatureType.SharpEdge, BladeFeatureType.BluntEdge);
        }
    }
}
