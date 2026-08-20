using System.Collections.Generic;
using BladeContact;
using UnityEngine;

/// <summary>
///     The L2 research-question layer: per-CONTACT tangential behaviour chosen from what the two touching
///     blade features actually ARE.
/// </summary>
/// <remarks>
///     <para>
///         Each fixed step, for every pair of registered blades PhysX reported touching, this runs one
///         closest-feature query against the authored shells, reads the two raw feature identities, maps
///         them to <see cref="BladeSemanticRegion"/> by authored designation only, classifies the contact
///         as a <see cref="BladeContactScenario"/>, resolves a <see cref="BladeTangentialPolicy"/>, and
///         adds an equal-and-opposite tangential force at the contact point. Nothing else is touched.
///     </para>
///     <para>
///         <b>What this is NOT.</b> It does not suppress, replace or correct PhysX contact. It does not
///         alter admissibility, time of impact, penetration or the contact normal. It does not write
///         collider <see cref="PhysicsMaterial"/> friction -- a collider material is one value for every
///         contact that collider is in, and a sword touches several things at once. It adds a term; PhysX
///         keeps its own.
///     </para>
///     <para>
///         <b>The parameter values are PROVISIONAL.</b> The two endpoint parameter sets in
///         <see cref="BladeTangentialPolicy"/> and the tangential compliance below are placeholders for
///         demonstrating the mechanism. They are NOT calibrated material friction, and no magnitude
///         measured through this layer is a physical claim about the specimen until they are declared and
///         calibrated.
///     </para>
/// </remarks>
[DisallowMultipleComponent]
public sealed class RqBladeTangentialSolver : MonoBehaviour
{
    /// <summary>Tangential state of one blade-vs-blade contact.</summary>
    public enum TangentialState
    {
        /// <summary>No PhysX contact between these two blades this step.</summary>
        Separated,

        /// <summary>Held: the tangential demand is inside the bind limit and the contact is not travelling.</summary>
        Bound,

        /// <summary>Broken away: the demand exceeded the bind limit and the contact is travelling.</summary>
        Slipping,

        /// <summary>The contact was live and then the normal load fell away.</summary>
        Released
    }

    /// <summary>Everything a readout needs about one contact, sampled after the last solve.</summary>
    public struct Snapshot
    {
        public bool Valid;
        public string LabelA;
        public string LabelB;

        // RAW layer -- reported unchanged, never reasoned backwards from.
        public BladeFeatureType RawTypeA;
        public BladeFeatureType RawTypeB;
        public string RawIdA;
        public string RawIdB;

        // SEMANTIC layer -- derived from authored designation only.
        public BladeSemanticRegion RegionA;
        public BladeSemanticRegion RegionB;
        public BladeContactScenario Scenario;
        public bool ClassificationValid;

        // Resolved policy for that scenario.
        public BladeTangentialParameters Parameters;
        public float EdgeNonCuttingBind;

        public TangentialState State;
        public float NormalForce;
        public float TangentialDemand;
        public float BindLimit;
        public float AppliedTangentialForce;
        public float BreakawayForce;
        public bool BreakawayRecorded;

        /// <summary>Tangential separation of the two anchored material points, metres.</summary>
        public float TangentialDisplacement;

        /// <summary>How far the contact has travelled ALONG each blade since it was first classified.</summary>
        public float DriftAlongA;
        public float DriftAlongB;

        public float TangentialSpeed;

        /// <summary>Most negative PhysX contact separation this step. Negative means overlap.</summary>
        public float SignedOverlap;

        public float ShellSeparation;

        /// <summary>The two PhysX colliders that actually produced the deepest contact this step.</summary>
        public string ContactColliderA;
        public string ContactColliderB;

        /// <summary>Their PhysicsMaterials, as PhysX will combine them. Reported, never written.</summary>
        public string ContactMaterialA;
        public string ContactMaterialB;

        public int ContactCount;
        /// <summary>
        ///     The step this pair's TANGENTIAL SOLVE last ran. This is a solve timestamp and says nothing
        ///     about how old the CLASSIFICATION is: the solve runs every step a contact is reported, while
        ///     the classifier runs on its own interval. Reading this as classification freshness is wrong,
        ///     and did mislead one measurement pass. Use <see cref="ClassifyAttemptedAtStep" /> and
        ///     <see cref="ClassWrittenAtStep" /> for that.
        /// </summary>
        public int LastSolvedStep;

        /// <summary>
        ///     The step the classifier was last INVOKED on this pair, whether or not it produced a usable
        ///     witness. Gated by <c>classificationInterval</c>, so consecutive steps normally share a value.
        /// </summary>
        public int ClassifyAttemptedAtStep;

        /// <summary>
        ///     The step a classification was last successfully WRITTEN for this pair, i.e. a usable witness
        ///     was found and the feature identities and scenario were replaced.
        /// </summary>
        /// <remarks>
        ///     When an attempt runs out of budget or yields no valid witness the pair KEEPS its previous
        ///     class and this value does not move, so <c>ClassifyAttemptedAtStep</c> being newer than this
        ///     is exactly the signature of a stale class being carried forward. The difference between this
        ///     and the reading step is the real age of the attribution, and it is the number a measurement
        ///     must quote rather than <see cref="LastSolvedStep" />.
        /// </remarks>
        public int ClassWrittenAtStep;

        /// <summary>Steps since this relationship was first seen in which PhysX reported it touching.</summary>
        public int TouchedSteps;

        /// <summary>Steps in which PhysX reported no contact between two blades that were engaged.</summary>
        /// <remarks>
        ///     A single-point contact on a cutting edge drops in and out of the narrow phase. Every step it
        ///     is missing, the measured normal load is zero and so is the bind limit that is proportional
        ///     to it. This counter is here so that is visible rather than inferred.
        /// </remarks>
        public int DropoutSteps;
    }

    private sealed class Pair
    {
        public RqBladeContact A;
        public RqBladeContact B;

        // Accumulated from the collision callbacks of the most recent completed step.
        public Vector3 PointSum;
        public Vector3 NormalSum;
        public Vector3 Impulse;
        public int ContactCount;
        public float SignedOverlap;
        public Collider DeepestA;
        public Collider DeepestB;
        public int ReportedStep = -1;

        // Stick anchors: one material point per body, in that body's own local space.
        public bool Anchored;
        public Vector3 AnchorLocalA;
        public Vector3 AnchorLocalB;

        public bool WitnessLatched;
        public Vector3 FirstWitnessLocalA;
        public Vector3 FirstWitnessLocalB;
        public Vector3 LatestWitnessLocalA;
        public Vector3 LatestWitnessLocalB;

        public TangentialState State = TangentialState.Separated;
        public int SlipCandidateSteps;
        public int EngagedSinceStep = -1;
        public float BreakawayForce;
        public bool BreakawayRecorded;
        public int NextClassifyStep;

        public readonly BladeShellScratch Scratch = new BladeShellScratch();
        public readonly HashSet<int> CountedCollisions = new HashSet<int>();
        public Snapshot Readout;
    }

    private readonly struct Key : System.IEquatable<Key>
    {
        private readonly int low;
        private readonly int high;

        public Key(RqBladeContact a, RqBladeContact b)
        {
            int x = a.GetInstanceID();
            int y = b.GetInstanceID();
            low = x < y ? x : y;
            high = x < y ? y : x;
        }

        public bool Equals(Key other) => low == other.low && high == other.high;
        public override bool Equals(object obj) => obj is Key other && Equals(other);
        public override int GetHashCode() => (low * 397) ^ high;
    }

    [Header("Behaviour")]
    [Tooltip("Off leaves every blade contact to ordinary PhysX, so the same rig can be run both ways.")]
    [SerializeField] private bool applyTangentialBehaviour = true;

    [Tooltip("PROVISIONAL. Tangential response endpoints per semantic scenario. Placeholders for " +
             "demonstrating the mechanism, NOT calibrated material friction.")]
    [SerializeField] private BladeTangentialPolicy policy = BladeTangentialPolicy.Default;

    [Header("Tangential compliance (PROVISIONAL model parameters)")]
    [Tooltip("N/m. How hard a held contact is pulled back to where it stuck. Sets how far a bound " +
             "contact creeps before it reaches its bind limit. A model parameter, not a measurement.")]
    [SerializeField] private float bindStiffness = 20000f;

    [Tooltip("N per m/s. Damping on the held contact, so the bind does not ring.")]
    [SerializeField] private float bindDamping = 60f;

    [Tooltip("Consecutive fixed steps the slip condition must hold before the contact is called broken. " +
             "A crossed knife-edge contact carries one contact point and jitters, and a single-step " +
             "excursion above the bind limit is that jitter, not a breakaway. This is hysteresis on the " +
             "state machine, not a physical threshold.")]
    [Min(1)]
    [SerializeField] private int slipDwellSteps = 4;

    [Header("Query")]
    [Tooltip("Fixed steps between closest-feature classifications. A new pair is always classified on " +
             "its first step, so a strike is classified on impact; after that the classification is held " +
             "and re-queried at this interval. The tangential force is applied EVERY step regardless. " +
             "At 200 Hz one closest-feature query on this specimen costs a few milliseconds, which is the " +
             "whole fixed-step budget. This is the dial that keeps the demonstration watchable; it is not " +
             "a claim that the contact only changes this often.")]
    [Min(1)]
    [SerializeField] private int classificationInterval = 20;

    [SerializeField] private BladeSweepSettings sweepSettings = BladeSweepSettings.Default;

    [Header("Diagnostics")]
    [SerializeField] private bool logFirstClassification = true;

    private readonly Dictionary<Key, Pair> pairs = new Dictionary<Key, Pair>();
    private readonly List<Key> scratchKeys = new List<Key>();
    private readonly List<ContactPoint> contactBuffer = new List<ContactPoint>(32);
    private readonly HashSet<Key> loggedPairs = new HashSet<Key>();

    private int step;

    /// <summary>Whether the semantic layer is currently adding anything at all.</summary>
    public bool ApplyTangentialBehaviour
    {
        get { return applyTangentialBehaviour; }
        set { applyTangentialBehaviour = value; }
    }

    /// <summary>The policy in force. Serialized, provisional, and restated in every readout.</summary>
    public BladeTangentialPolicy Policy { get { return policy; } }

    public float BindStiffness { get { return bindStiffness; } }
    public float BindDamping { get { return bindDamping; } }

    /// <summary>Milliseconds spent in closest-feature queries during the last solved step.</summary>
    public double LastQueryMs { get; private set; }

    /// <summary>Blade-vs-blade contacts solved during the last step.</summary>
    public int LastSolvedPairs { get; private set; }

    /// <summary>
    ///     Every relationship the solver currently holds, most recently solved first.
    /// </summary>
    /// <remarks>
    ///     For free-play readouts, where nothing knows in advance which two blades will meet. Pairs that
    ///     have separated are still listed, with the state they ended in, until one of the blades leaves.
    /// </remarks>
    public void CollectSnapshots(List<Snapshot> into)
    {
        into.Clear();
        foreach (KeyValuePair<Key, Pair> entry in pairs)
            if (entry.Value.Readout.Valid) into.Add(entry.Value.Readout);

        into.Sort((x, y) => y.LastSolvedStep.CompareTo(x.LastSolvedStep));
    }

    /// <summary>Latest state of the contact between two specific blades, for a readout.</summary>
    public bool TryGetSnapshot(RqBladeContact a, RqBladeContact b, out Snapshot snapshot)
    {
        Pair pair;
        if (a != null && b != null && pairs.TryGetValue(new Key(a, b), out pair))
        {
            snapshot = pair.Readout;
            return snapshot.Valid;
        }

        snapshot = default(Snapshot);
        return false;
    }

    /// <summary>
    ///     Re-establishes where a contact is held from, and clears its breakaway record.
    /// </summary>
    /// <remarks>
    ///     Called when a rig latches its measurement zero. Without it the bind is anchored wherever the
    ///     blades first touched, which for a knife-edge contact is a metastable apex-on-apex pose that
    ///     beds in millimetres later. Anchoring at the measurement zero instead means the reported
    ///     displacement and the held-contact stretch start counting from the same instant.
    /// </remarks>
    public void ReAnchor(RqBladeContact a, RqBladeContact b)
    {
        Pair pair;
        if (a == null || b == null || !pairs.TryGetValue(new Key(a, b), out pair)) return;

        pair.Anchored = false;
        pair.WitnessLatched = false;
        pair.SlipCandidateSteps = 0;
        pair.Readout.TouchedSteps = 0;
        pair.Readout.DropoutSteps = 0;
        pair.BreakawayRecorded = false;
        pair.BreakawayForce = 0f;
        pair.Readout.DriftAlongA = 0f;
        pair.Readout.DriftAlongB = 0f;
    }

    /// <summary>Drops every relationship a blade is in, when it leaves the scene.</summary>
    public void Forget(RqBladeContact blade)
    {
        scratchKeys.Clear();
        foreach (KeyValuePair<Key, Pair> entry in pairs)
        {
            if (entry.Value.A == blade || entry.Value.B == blade) scratchKeys.Add(entry.Key);
        }

        for (int i = 0; i < scratchKeys.Count; i++) pairs.Remove(scratchKeys[i]);
    }

    /// <summary>
    ///     Records one PhysX collision. Only contacts where BOTH colliders are registered blades are kept;
    ///     everything else -- guard, grip, pommel, floor, props, unregistered swords -- is ignored here and
    ///     left to PhysX untouched.
    /// </summary>
    internal void ReportCollision(RqBladeContact reporter, Collision collision)
    {
        int count = collision.GetContacts(contactBuffer);
        if (count == 0) return;

        for (int i = 0; i < count; i++)
        {
            ContactPoint contact = contactBuffer[i];
            if (!reporter.Owns(contact.thisCollider)) continue;

            RqBladeContact other;
            if (!RqBladeContact.TryResolve(contact.otherCollider, out other)) continue;
            if (other == reporter) continue;

            Key key = new Key(reporter, other);
            Pair pair;
            if (!pairs.TryGetValue(key, out pair))
            {
                pair = new Pair { A = reporter, B = other };
                pairs.Add(key, pair);
            }

            // Both blades receive a callback for the same contact. Keep one side's view of it so the
            // contact count and the impulse are not counted twice.
            if (pair.A != reporter) continue;

            if (pair.ReportedStep != step)
            {
                pair.ReportedStep = step;
                pair.PointSum = Vector3.zero;
                pair.NormalSum = Vector3.zero;
                pair.ContactCount = 0;
                pair.SignedOverlap = float.MaxValue;
                pair.Impulse = Vector3.zero;
                pair.DeepestA = null;
                pair.DeepestB = null;
                pair.CountedCollisions.Clear();
            }

            // One blade can carry several shapes, so PhysX delivers a separate Collision per COLLIDER pair
            // in the same step, each with its own impulse. They are summed once per collider pair -- not
            // once per contact point inside it, and not keyed on the Collision object, which Unity reuses.
            int shapePair = contact.thisCollider.GetInstanceID() * 397 ^ contact.otherCollider.GetInstanceID();
            if (pair.CountedCollisions.Add(shapePair)) pair.Impulse += collision.impulse;

            pair.PointSum += contact.point;
            pair.NormalSum += contact.normal;
            pair.ContactCount++;

            if (contact.separation < pair.SignedOverlap)
            {
                pair.SignedOverlap = contact.separation;
                pair.DeepestA = contact.thisCollider;
                pair.DeepestB = contact.otherCollider;
            }
        }
    }

    private void FixedUpdate()
    {
        step++;
        LastSolvedPairs = 0;

        if (pairs.Count == 0)
        {
            LastQueryMs = 0.0;
            return;
        }

        System.Diagnostics.Stopwatch clock = System.Diagnostics.Stopwatch.StartNew();
        double queryMs = 0.0;

        scratchKeys.Clear();
        foreach (KeyValuePair<Key, Pair> entry in pairs) scratchKeys.Add(entry.Key);

        for (int i = 0; i < scratchKeys.Count; i++)
        {
            Key key = scratchKeys[i];
            Pair pair = pairs[key];

            if (pair.A == null || pair.B == null || pair.A.Body == null || pair.B.Body == null)
            {
                pairs.Remove(key);
                continue;
            }

            // The callbacks for step N arrive after step N is simulated, so they are consumed at the top
            // of step N+1 -- which is also the moment the poses they refer to are the current ones.
            bool touching = pair.ReportedStep == step - 1 && pair.ContactCount > 0;

            if (!touching)
            {
                if (pair.State == TangentialState.Bound || pair.State == TangentialState.Slipping)
                    pair.State = TangentialState.Released;

                pair.Anchored = false;
                pair.Readout.State = pair.State;
                pair.Readout.NormalForce = 0f;
                pair.Readout.AppliedTangentialForce = 0f;
                pair.Readout.TangentialDemand = 0f;
                pair.Readout.ContactCount = 0;

                // A dropout only counts while the pair was engaged. Once it has genuinely separated and
                // been released, absence is not a dropout.
                if (pair.EngagedSinceStep >= 0 && pair.State == TangentialState.Released)
                    pair.EngagedSinceStep = -1;
                else if (pair.EngagedSinceStep >= 0)
                    pair.Readout.DropoutSteps++;

                continue;
            }

            double before = clock.Elapsed.TotalMilliseconds;
            Solve(key, pair, clock);
            queryMs += clock.Elapsed.TotalMilliseconds - before;
            LastSolvedPairs++;
        }

        LastQueryMs = queryMs;
    }

    private void Solve(Key key, Pair pair, System.Diagnostics.Stopwatch clock)
    {
        float dt = Time.fixedDeltaTime;
        Vector3 point = pair.PointSum / pair.ContactCount;
        Vector3 normal = pair.NormalSum.sqrMagnitude > 1e-12f ? pair.NormalSum.normalized : Vector3.up;

        // Classify. The raw witness is reported unchanged; the semantic layer is derived from the
        // authored designation of those two features and from nothing else.
        if (step >= pair.NextClassifyStep)
        {
            pair.NextClassifyStep = step + classificationInterval;

            // Recorded whether or not the attempt yields a usable witness. An attempt that fails leaves
            // ClassWrittenAtStep behind, which is what makes a carried-forward class visible as stale.
            pair.Readout.ClassifyAttemptedAtStep = step;
            Classify(key, pair, clock);
        }

        UpdateDrift(pair);

        BladeTangentialParameters prm = pair.Readout.ClassificationValid
            ? policy.Resolve(pair.Readout.Scenario)
            : default(BladeTangentialParameters);

        // PhysX reports the impulse it needed to resolve this pair; over the step that is the normal load
        // holding the two blades together. Magnitude only, so the callback's sign convention cannot leak in.
        float normalForce = Mathf.Abs(Vector3.Dot(pair.Impulse, normal)) / Mathf.Max(dt, 1e-6f);

        Rigidbody bodyA = pair.A.Body;
        Rigidbody bodyB = pair.B.Body;

        if (!pair.Anchored)
        {
            pair.AnchorLocalA = bodyA.transform.InverseTransformPoint(point);
            pair.AnchorLocalB = bodyB.transform.InverseTransformPoint(point);
            pair.Anchored = true;
            pair.State = TangentialState.Bound;
            pair.BreakawayRecorded = false;
            pair.BreakawayForce = 0f;
        }

        Vector3 anchorA = bodyA.transform.TransformPoint(pair.AnchorLocalA);
        Vector3 anchorB = bodyB.transform.TransformPoint(pair.AnchorLocalB);

        Vector3 offset = anchorB - anchorA;
        Vector3 tangentialOffset = offset - normal * Vector3.Dot(offset, normal);

        Vector3 relative = bodyB.GetPointVelocity(point) - bodyA.GetPointVelocity(point);
        Vector3 tangentialVelocity = relative - normal * Vector3.Dot(relative, normal);

        // The force that would hold this contact exactly where it stuck.
        Vector3 demand = -(bindStiffness * tangentialOffset + bindDamping * tangentialVelocity);
        float bindLimit = prm.StaticBindThreshold * normalForce;
        float slideLimit = prm.DynamicFriction * normalForce;

        Vector3 force = Vector3.zero;

        if (pair.State == TangentialState.Bound)
        {
            bool overLimit = demand.magnitude > bindLimit &&
                             tangentialVelocity.magnitude > prm.ReleaseThreshold;

            pair.SlipCandidateSteps = overLimit ? pair.SlipCandidateSteps + 1 : 0;

            if (pair.SlipCandidateSteps >= slipDwellSteps)
            {
                pair.State = TangentialState.Slipping;
                pair.SlipCandidateSteps = 0;
                if (!pair.BreakawayRecorded)
                {
                    pair.BreakawayForce = bindLimit;
                    pair.BreakawayRecorded = true;
                }
            }
            else
            {
                force = demand.magnitude > bindLimit ? demand.normalized * bindLimit : demand;
            }
        }

        if (pair.State == TangentialState.Slipping)
        {
            if (tangentialVelocity.sqrMagnitude > 1e-12f)
                force = -tangentialVelocity.normalized * slideLimit;

            if (tangentialVelocity.magnitude < prm.ReleaseThreshold)
            {
                pair.AnchorLocalA = bodyA.transform.InverseTransformPoint(point);
                pair.AnchorLocalB = bodyB.transform.InverseTransformPoint(point);
                pair.State = TangentialState.Bound;
                pair.SlipCandidateSteps = 0;
            }
        }

        if (applyTangentialBehaviour && pair.Readout.ClassificationValid && normalForce > 0f)
        {
            bodyB.AddForceAtPosition(force, point, ForceMode.Force);
            bodyA.AddForceAtPosition(-force, point, ForceMode.Force);
        }
        else
        {
            force = Vector3.zero;
        }

        pair.Readout.Valid = true;
        pair.Readout.LabelA = pair.A.Label;
        pair.Readout.LabelB = pair.B.Label;
        pair.Readout.Parameters = prm;
        pair.Readout.EdgeNonCuttingBind = policy.EdgeNonCuttingBind;
        pair.Readout.State = pair.State;
        pair.Readout.NormalForce = normalForce;
        pair.Readout.TangentialDemand = demand.magnitude;
        pair.Readout.BindLimit = bindLimit;
        pair.Readout.AppliedTangentialForce = force.magnitude;
        pair.Readout.BreakawayForce = pair.BreakawayForce;
        pair.Readout.BreakawayRecorded = pair.BreakawayRecorded;
        pair.Readout.TangentialDisplacement = tangentialOffset.magnitude;
        pair.Readout.TangentialSpeed = tangentialVelocity.magnitude;
        pair.Readout.SignedOverlap = pair.SignedOverlap;
        pair.Readout.ContactCount = pair.ContactCount;
        pair.Readout.ContactColliderA = Describe(pair.DeepestA);
        pair.Readout.ContactColliderB = Describe(pair.DeepestB);
        pair.Readout.ContactMaterialA = DescribeMaterial(pair.DeepestA);
        pair.Readout.ContactMaterialB = DescribeMaterial(pair.DeepestB);
        pair.Readout.LastSolvedStep = step;
        pair.Readout.TouchedSteps++;
        if (pair.EngagedSinceStep < 0) pair.EngagedSinceStep = step;
    }

    /// <summary>Names a collider for the readout, so which SHAPE is in contact is never inferred.</summary>
    private static string Describe(Collider collider) =>
        collider == null ? "(none)" : collider.name;

    /// <summary>
    ///     Names a collider's PhysicsMaterial and its coefficients, for the readout only.
    /// </summary>
    /// <remarks>
    ///     Reported, never written. A null material means the collider is running PhysX's built-in
    ///     defaults, which is a real and undeclared value and should be visible as such.
    /// </remarks>
    private static string DescribeMaterial(Collider collider)
    {
        if (collider == null) return "(none)";

        PhysicsMaterial material = collider.sharedMaterial;
        if (material == null) return "no material (PhysX built-in 0.6/0.6)";

        return material.name + " s" + material.staticFriction.ToString("F2") +
               "/d" + material.dynamicFriction.ToString("F2") + " " + material.frictionCombine;
    }

    private void Classify(Key key, Pair pair, System.Diagnostics.Stopwatch clock)
    {
        BladePose poseA = pair.A.Pose;
        BladePose poseB = pair.B.Pose;

        BladeFeaturePair witness;
        bool ok = BladeShellSweep.TryClosestFeaturePair(
            pair.A.Data, poseA, pair.B.Data, poseB,
            sweepSettings, pair.Scratch, null, clock, out witness);

        if (!ok || !witness.FeatureA.IsValid || !witness.FeatureB.IsValid)
        {
            // A query that ran out of budget carries no usable witness. Rather than guess, the pair keeps
            // whatever it was last classified as; the readout still shows when that was.
            return;
        }

        BladeFeatureType typeA = pair.A.Data.TypeOf(witness.FeatureA);
        BladeFeatureType typeB = pair.B.Data.TypeOf(witness.FeatureB);

        pair.Readout.RawTypeA = typeA;
        pair.Readout.RawTypeB = typeB;
        pair.Readout.RawIdA = pair.A.Data.IdOf(witness.FeatureA);
        pair.Readout.RawIdB = pair.B.Data.IdOf(witness.FeatureB);
        pair.Readout.RegionA = BladeContactScenarios.RegionOf(typeA);
        pair.Readout.RegionB = BladeContactScenarios.RegionOf(typeB);
        pair.Readout.Scenario = BladeContactScenarios.Classify(typeA, typeB);
        pair.Readout.ClassificationValid = true;

        // Only reached when a usable witness replaced the feature identities above. An attempt that
        // returned early left this behind, so (readingStep - ClassWrittenAtStep) is the attribution's
        // true age.
        pair.Readout.ClassWrittenAtStep = step;
        pair.Readout.ShellSeparation = witness.Separation;

        // Against the transform the shell query was posed with, which is the body actually carrying the
        // blade. Using the authored transform here would express the witness in a frame the contact is
        // not in whenever an attachment system is hosting the sword's physics.
        pair.LatestWitnessLocalA = pair.A.ActiveTransform.InverseTransformPoint(witness.WitnessA);
        pair.LatestWitnessLocalB = pair.B.ActiveTransform.InverseTransformPoint(witness.WitnessB);

        if (!pair.WitnessLatched)
        {
            pair.FirstWitnessLocalA = pair.LatestWitnessLocalA;
            pair.FirstWitnessLocalB = pair.LatestWitnessLocalB;
            pair.WitnessLatched = true;

            if (logFirstClassification && loggedPairs.Add(key))
            {
                Debug.Log(
                    "[L2] first contact " + pair.A.Label + " x " + pair.B.Label + ": " +
                    BladeContactScenarios.Describe(typeA, pair.Readout.RawIdA, typeB, pair.Readout.RawIdB) +
                    "  policy " + policy.Describe(pair.Readout.Scenario));
            }
        }
    }

    /// <summary>
    ///     How far the witness has travelled ALONG each blade since the contact was first classified.
    /// </summary>
    /// <remarks>
    ///     This is the blade-local statement of the bind. A world-space displacement cannot separate a
    ///     contact that is held on both blades while the blades themselves move from one that is sliding
    ///     down them.
    /// </remarks>
    private static void UpdateDrift(Pair pair)
    {
        if (!pair.WitnessLatched) return;

        pair.Readout.DriftAlongA = Vector3.Dot(
            pair.LatestWitnessLocalA - pair.FirstWitnessLocalA, pair.A.LocalLongAxis);
        pair.Readout.DriftAlongB = Vector3.Dot(
            pair.LatestWitnessLocalB - pair.FirstWitnessLocalB, pair.B.LocalLongAxis);
    }
}
