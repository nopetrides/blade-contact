using UnityEngine;

namespace BladeContact
{
    public enum BladeContactStatus : byte
    {
        /// <summary>The requested motion completes without the shells reaching contact separation.</summary>
        NoContact,

        /// <summary>First contact was located at <see cref="BladeShellContact.TimeOfImpact"/>.</summary>
        Contact,

        /// <summary>Advancement did not converge within the iteration ceiling; motion is still blocked.</summary>
        IterationLimit
    }

    /// <summary>
    /// Closest authored feature pair between two shells at one pose, with the witness point on each.
    /// </summary>
    public readonly struct BladeFeaturePair
    {
        public readonly BladeFeatureRef FeatureA;
        public readonly BladeFeatureRef FeatureB;
        public readonly Vector3 WitnessA;
        public readonly Vector3 WitnessB;
        public readonly float Separation;

        public BladeFeaturePair(
            BladeFeatureRef featureA, BladeFeatureRef featureB,
            Vector3 witnessA, Vector3 witnessB, float separation)
        {
            FeatureA = featureA;
            FeatureB = featureB;
            WitnessA = witnessA;
            WitnessB = witnessB;
            Separation = separation;
        }

        /// <summary>Unit normal from shell A toward shell B. Zero when the witnesses coincide.</summary>
        public Vector3 Normal
        {
            get
            {
                Vector3 delta = WitnessB - WitnessA;
                return delta.sqrMagnitude > 1e-18f ? delta.normalized : Vector3.zero;
            }
        }

        public static BladeFeaturePair None =>
            new BladeFeaturePair(BladeFeatureRef.None, BladeFeatureRef.None, Vector3.zero, Vector3.zero, float.MaxValue);
    }

    /// <summary>Outcome of one first-contact query between a registered shell pair.</summary>
    public readonly struct BladeShellContact
    {
        public readonly BladeContactStatus Status;

        /// <summary>Fraction of the requested motion that may be accepted, in [0, 1].</summary>
        public readonly float TimeOfImpact;

        public readonly BladeFeaturePair Pair;

        /// <summary>Conservative-advancement iterations consumed, for the event trace.</summary>
        public readonly int Iterations;

        public BladeShellContact(
            BladeContactStatus status, float timeOfImpact, BladeFeaturePair pair, int iterations)
        {
            Status = status;
            TimeOfImpact = timeOfImpact;
            Pair = pair;
            Iterations = iterations;
        }

        /// <summary>True when the requested motion must not be accepted in full.</summary>
        public bool BlocksMotion => Status != BladeContactStatus.NoContact;
    }
}
