package frc.log.lib;

@FunctionalInterface
public interface RelativeTimeSource {
  public long convertToRelativeNanos(long nanos);
}
