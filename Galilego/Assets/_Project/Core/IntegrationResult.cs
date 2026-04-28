namespace Galilego.Physics
{
    public struct IntegrationResult
    {
        public Vector3d Position { get; }
        public Vector3d Velocity { get; }

        public IntegrationResult(Vector3d position, Vector3d velocity)
        {
            Position = position;
            Velocity = velocity;
        }
    }
}
