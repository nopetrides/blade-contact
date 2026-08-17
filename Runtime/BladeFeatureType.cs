namespace BladeContact
{
    /// <summary>Semantic type authored into a <see cref="BladeShell"/> feature.</summary>
    public enum BladeFeatureType : byte
    {
        BroadFace,
        BevelFace,
        SharpEdge,
        BluntEdge,
        Tip,
        Unresolved
    }
}
