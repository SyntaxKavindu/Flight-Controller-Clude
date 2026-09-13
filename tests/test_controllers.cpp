/*
 * The attitude controller, driven directly.
 *
 * Links NO stub/: this loop takes dt as a parameter and touches no peripheral,
 * so if this ever starts needing stub/ a board dependency has crept into the
 * control law.
 *
 * Conventions (see tests/README.md): body FRD, earth NED, quaternions are
 * body -> NED, Euler triples are ZYX.
 */

#include "Attitude.hpp"
#include "test.hpp"

namespace {

const float D2R = phys::D2R;
const float R2D = phys::R2D;

float qlen(const Quaternionf &q)
{
	return std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

Quaternionf qnorm(const Quaternionf &q)
{
	const float n = qlen(q);
	return Quaternionf(q.w/n, q.x/n, q.y/n, q.z/n);
}

// Advance an attitude by a body-frame angular rate for dt. First order, which
// is all these tests need at 400 Hz.
Quaternionf integrate(const Quaternionf &q, const Vector3f &w, float dt)
{
	return qnorm(q * Quaternionf(1.0f, w.x*dt*0.5f, w.y*dt*0.5f, w.z*dt*0.5f));
}

// Compass bearing of a horizontal direction, degrees in [0, 360).
float bearing(const Vector3f &v)
{
	float b = std::atan2(v.y, v.x) * R2D;
	return (b < 0.0f) ? b + 360.0f : b;
}

// A unit thrust vector leaning `lean` radians toward compass `brg` degrees.
Vector3f leanToward(float lean, float brg_deg)
{
	const float b = brg_deg * D2R;
	return Vector3f(std::sin(lean) * std::cos(b),
	                std::sin(lean) * std::sin(b),
	                -std::cos(lean));
}

float angleBetween(const Vector3f &a, const Vector3f &b)
{
	const float d = a.dot(b) / (a.length() * b.length());
	return std::acos(d < -1.0f ? -1.0f : (d > 1.0f ? 1.0f : d)) * R2D;
}

} // namespace

int main(void)
{
	// -----------------------------------------------------------------------
	section("Euler <-> quaternion agrees with the estimator's convention");
	// -----------------------------------------------------------------------
	{
		// attitudeFromEuler must be ESEKF::quaternionFromEuler. Checked against
		// test.hpp's independent DCM rather than against itself.
		const float r = 12.0f*D2R, p = -20.0f*D2R, y = 100.0f*D2R;
		const Quaternionf q = Attitude::attitudeFromEuler(r, p, y);

		float R[3][3];
		phys::dcm(r, p, y, R);
		const Vector3f fwd = q.rotate(Vector3f(1.0f, 0.0f, 0.0f));

		checkNear(fwd.x, R[0][0], 1e-5f, "body X -> NED matches the DCM (north)");
		checkNear(fwd.y, R[1][0], 1e-5f, "body X -> NED matches the DCM (east)");
		checkNear(fwd.z, R[2][0], 1e-5f, "body X -> NED matches the DCM (down)");
		checkNear(qlen(q), 1.0f, 1e-6f, "the quaternion is unit length");
		checkNear(Attitude::headingFromAttitude(q)*R2D, 100.0f, 1e-3f,
				"headingFromAttitude recovers the ZYX yaw");
	}

	// -----------------------------------------------------------------------
	section("A thrust vector carries no heading");
	// -----------------------------------------------------------------------
	{
		// THE property the whole input API rests on: the thrust direction is a
		// constraint, and the heading only picks which (roll, pitch) realises
		// it. Sweep the heading right round and the direction must not move.
		const Vector3f t = leanToward(10.0f*D2R, 0.0f);   // 10 deg, NORTH

		float worst = 0.0f;
		for (int h = 0; h < 360; h += 15) {
			const Quaternionf q = Attitude::attitudeFromThrustVector(t, h*D2R);
			const float err = angleBetween(Attitude::thrustVectorOf(q), t);
			if (err > worst) worst = err;
		}
		checkNear(worst, 0.0f, 1e-3f,
				"thrust direction is identical at every heading (worst, deg)");

		// The specific decomposition, because it is the thing that looks wrong
		// the first time: same world direction, pitch at one heading and roll
		// at another.
		const Quaternionf q0  = Attitude::attitudeFromThrustVector(t, 0.0f);
		const Quaternionf q90 = Attitude::attitudeFromThrustVector(t, 90.0f*D2R);
		// Compared as directions, not bearings: a bearing wraps, and "north"
		// is as likely to read 359.999 as 0.001.
		checkNear(angleBetween(Attitude::thrustVectorOf(q0),  t), 0.0f, 1e-3f,
				"heading 0: thrust points north");
		checkNear(angleBetween(Attitude::thrustVectorOf(q90), t), 0.0f, 1e-3f,
				"heading 90: thrust points STILL north");

		// Cross-check the decomposition through setEuler, which is the other
		// parameterisation of the same attitude.
		const Quaternionf e0  = Attitude::attitudeFromEuler(0.0f, -10.0f*D2R, 0.0f);
		const Quaternionf e90 = Attitude::attitudeFromEuler(-10.0f*D2R, 0.0f, 90.0f*D2R);
		checkNear(angleBetween(Attitude::thrustVectorOf(e0),  t), 0.0f, 1e-3f,
				"heading 0 is pitch -10");
		checkNear(angleBetween(Attitude::thrustVectorOf(e90), t), 0.0f, 1e-3f,
				"heading 90 is roll -10, same direction");
	}

	// -----------------------------------------------------------------------
	section("Degenerate thrust vectors are solved, not NaN'd");
	// -----------------------------------------------------------------------
	{
		// 90 deg lean -- thrust horizontal, the ZYX pitch singularity.
		const Vector3f side = leanToward(90.0f*D2R, 0.0f);
		const Quaternionf q = Attitude::attitudeFromThrustVector(side, 0.0f);
		check(std::isfinite(q.w) && std::isfinite(q.x)
		   && std::isfinite(q.y) && std::isfinite(q.z),
				"a 90 deg lean solves to a finite quaternion");
		checkNear(angleBetween(Attitude::thrustVectorOf(q), side), 0.0f, 1e-2f,
				"...and points where it was asked to");

		// Zero length: no direction to honour, so identity rather than a NaN.
		const Quaternionf z = Attitude::attitudeFromThrustVector(Vector3f(), 0.0f);
		checkNear(z.w, 1.0f, 1e-6f, "a zero-length thrust vector gives identity");
	}

	// -----------------------------------------------------------------------
	section("rotationVector always takes the short way round");
	// -----------------------------------------------------------------------
	{
		const Vector3f a = Attitude::rotationVector(
				Attitude::attitudeFromEuler(0.0f, 0.0f, 179.0f*D2R));
		const Vector3f b = Attitude::rotationVector(
				Attitude::attitudeFromEuler(0.0f, 0.0f, -179.0f*D2R));
		checkNear(a.z*R2D,  179.0f, 1e-2f, "+179 deg is +179");
		checkNear(b.z*R2D, -179.0f, 1e-2f, "-179 deg is -179, not +181");

		const Vector3f tiny = Attitude::rotationVector(
				Attitude::attitudeFromEuler(0.0f, 0.0f, 1e-5f));
		checkNear(tiny.z, 1e-5f, 1e-7f, "a tiny rotation does not divide by zero");
	}

	// -----------------------------------------------------------------------
	section("decompose splits tilt from heading, exactly");
	// -----------------------------------------------------------------------
	{
		Vector3f tilt; float yaw;

		// A pure heading error must leave the thrust axis alone.
		Attitude::decompose(Attitude::attitudeFromEuler(0.0f, 0.0f, 30.0f*D2R),
				tilt, yaw);
		checkNear(std::sqrt(tilt.x*tilt.x + tilt.y*tilt.y), 0.0f, 1e-5f,
				"pure yaw error -> zero tilt");
		checkNear(yaw*R2D, 30.0f, 1e-3f, "pure yaw error -> 30 deg of heading");

		// A pure tilt error must leave the heading alone.
		Attitude::decompose(Attitude::attitudeFromEuler(20.0f*D2R, 0.0f, 0.0f),
				tilt, yaw);
		checkNear(tilt.x*R2D, 20.0f, 1e-3f, "pure roll error -> 20 deg of tilt");
		checkNear(yaw*R2D, 0.0f, 1e-3f, "pure roll error -> zero heading");

		// Mixed: the tilt part never has a Z component, by construction.
		Attitude::decompose(
				Attitude::attitudeFromEuler(15.0f*D2R, -25.0f*D2R, 40.0f*D2R),
				tilt, yaw);
		checkNear(tilt.z, 0.0f, 1e-6f, "the tilt part has no Z component");

		// Inverted: not unique, but must be finite and 180 deg.
		Attitude::decompose(Attitude::attitudeFromEuler(180.0f*D2R, 0.0f, 0.0f),
				tilt, yaw);
		checkNear(std::sqrt(tilt.x*tilt.x + tilt.y*tilt.y)*R2D, 180.0f, 1e-2f,
				"an inverted target is 180 deg of tilt, not a NaN");
	}

	// -----------------------------------------------------------------------
	section("stoppingRate");
	// -----------------------------------------------------------------------
	{
		checkNear(Attitude::stoppingRate(0.5f, 8.0f), 2.828427f, 1e-4f,
				"sqrt(2 * a * angle)");
		checkNear(Attitude::stoppingRate(-0.5f, 8.0f), 2.828427f, 1e-4f,
				"sign of the error does not matter");
		checkNear(Attitude::stoppingRate(1.0f, 0.0f), 0.0f, 1e-6f,
				"a non-positive accel limit gives zero, not a NaN");
		checkNear(Attitude::stoppingRate(0.0f, 8.0f), 0.0f, 1e-6f,
				"zero error, zero allowance");
	}

	// -----------------------------------------------------------------------
	section("wrapPi terminates on every finite input");
	// -----------------------------------------------------------------------
	{
		checkNear(Attitude::wrapPi(0.5f), 0.5f, 1e-6f, "in range, unchanged");
		checkNear(Attitude::wrapPi(3.5f), 3.5f - 2.0f*3.14159265f, 1e-5f,
				"just over pi wraps negative");
		checkNear(Attitude::wrapPi(-3.5f), -3.5f + 2.0f*3.14159265f, 1e-5f,
				"just under -pi wraps positive");
		checkNear(Attitude::wrapPi(2.0f*3.14159265f + 1.0f), 1.0f, 1e-4f,
				"a full turn plus 1 rad wraps to 1 rad");
		checkNear(Attitude::wrapPi(-6.0f*3.14159265f - 1.0f), -1.0f, 1e-4f,
				"three turns the other way, likewise");
		// Exactly +/-pi is the boundary, where both signs name the same angle,
		// so only the magnitude is meaningful.
		checkNear(std::fabs(Attitude::wrapPi(7.0f*3.14159265f)), 3.14159265f,
				1e-4f, "an odd multiple of pi lands on the boundary");

		// The one that matters: a naive `while (rad > pi) rad -= 2*pi` never
		// terminates here, because 2*pi is below the ULP at this magnitude.
		// If this check is reached at all, the loop returned.
		const float huge = Attitude::wrapPi(1e30f);
		check(std::isfinite(huge) && std::fabs(huge) <= 3.14159266f,
				"a huge finite input RETURNS, in range");
		const float huge_neg = Attitude::wrapPi(-1e30f);
		check(std::isfinite(huge_neg) && std::fabs(huge_neg) <= 3.14159266f,
				"...and so does a huge negative one");
		checkNear(Attitude::wrapPi(std::nanf("")), 0.0f, 1e-9f,
				"a non-finite input gives zero rather than spinning");

		// Reached through the public API, which is how a broken mode layer
		// would actually deliver it.
		Attitude att;
		att.reset(Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f));
		att.setEuler(0.0f, 0.0f, 1e30f);
		check(std::isfinite(att.getHeadingTarget()),
				"a garbage heading through setEuler() does not hang the loop");
	}

	// -----------------------------------------------------------------------
	section("Engaging");
	// -----------------------------------------------------------------------
	{
		Attitude att;
		check(!att.isActive(), "a fresh controller is NOT engaged");

		const Quaternionf here = Attitude::attitudeFromEuler(5.0f*D2R, -3.0f*D2R,
				77.0f*D2R);
		att.setEuler(0.0f, 0.0f, 0.0f);       // a target set before engaging
		att.update(here, 0.0025f);
		checkNear(att.getRateTarget().length(), 0.0f, 1e-9f,
				"a disengaged loop commands ZERO, not the pending correction");

		att.reset(here);
		check(att.isActive(), "reset() engages it");
		att.update(here, 0.0025f);
		checkNear(att.getTiltError()*R2D, 0.0f, 1e-3f,
				"reset() snaps the target to where the aircraft already is (tilt)");
		checkNear(att.getYawError()*R2D, 0.0f, 1e-3f, "...and the heading");
		checkNear(att.getRateTarget().length(), 0.0f, 1e-5f,
				"...so it commands nothing");
		checkNear(att.getHeadingTarget()*R2D, 77.0f, 1e-2f,
				"the heading target is seeded from the measurement");

		Attitude bad;
		bad.reset(Quaternionf(std::nanf(""), 0.0f, 0.0f, 0.0f));
		check(!bad.isActive(),
				"reset() with a non-finite attitude leaves it DISENGAGED");
	}

	// -----------------------------------------------------------------------
	section("The two setters agree when the frames agree");
	// -----------------------------------------------------------------------
	{
		Attitude a, b;
		const Quaternionf level = Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f);
		a.reset(level);
		b.reset(level);

		const float h = 35.0f*D2R;
		a.setEuler(8.0f*D2R, -12.0f*D2R, h);
		b.setThrustVectorHeading(
				Attitude::thrustVectorOf(
						Attitude::attitudeFromEuler(8.0f*D2R, -12.0f*D2R, h)), h);

		checkNear(angleBetween(a.getThrustVectorTarget(),
				b.getThrustVectorTarget()), 0.0f, 1e-3f,
				"same thrust direction from both setters");
		checkNear(a.getHeadingTarget(), b.getHeadingTarget(), 1e-5f,
				"same heading target from both setters");
	}

	// -----------------------------------------------------------------------
	section("A heading target that leads the aircraft cannot move the thrust");
	// -----------------------------------------------------------------------
	{
		// The exact case that motivated this API. Measured heading 90, stick
		// rotated by 90 into a thrust vector pointing EAST, and a heading
		// target 26 deg ahead because the yaw stick is deflected.
		Attitude att;
		const Quaternionf meas = Attitude::attitudeFromEuler(0.0f, 0.0f, 90.0f*D2R);
		att.reset(meas);

		const Vector3f t_east = leanToward(10.0f*D2R, 90.0f);
		att.setThrustVectorHeading(t_east, 116.0f*D2R);
		att.update(meas, 0.0025f);

		checkNear(bearing(att.getThrustVectorTarget()), 90.0f, 1e-2f,
				"the target still points EAST, with a 26 deg heading lead");
		// 25.66, not 26: decompose() returns the rotation about BODY z, and
		// once the airframe is tilted that is not the same angle as the
		// difference of two ZYX yaws. The point is that the gap lands here
		// rather than in the tilt, not that it is numerically identical.
		checkNear(att.getYawError()*R2D, 25.66f, 0.2f,
				"the whole gap lands in the HEADING error");

		// And the error axis is the thrust axis itself -- which is why a
		// rotation of that size cannot disturb where the thrust points.
		const Vector3f err = att.getAttitudeError();
		checkNear(std::sqrt(err.x*err.x + err.y*err.y)*R2D, 10.0f, 0.5f,
				"the tilt error is the 10 deg lean, and nothing else");
	}

	// -----------------------------------------------------------------------
	section("The control law");
	// -----------------------------------------------------------------------
	{
		Attitude att;
		const Quaternionf level = Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f);
		att.reset(level);

		// 5 deg of roll error, well inside every limit -> P * error.
		att.setEuler(5.0f*D2R, 0.0f, 0.0f);
		att.update(level, 0.0025f);
		checkNear(att.getRateTarget().x, ATTITUDE_P_ROLL * 5.0f*D2R, 1e-4f,
				"roll rate is P times the roll error");
		checkNear(att.getRateTarget().y, 0.0f, 1e-5f, "...and nothing on pitch");
		checkNear(att.getRateTarget().z, 0.0f, 1e-5f, "...or yaw");

		// 5 deg of heading error.
		att.setEuler(0.0f, 0.0f, 5.0f*D2R);
		att.update(level, 0.0025f);
		checkNear(att.getRateTarget().z, ATTITUDE_P_YAW * 5.0f*D2R, 1e-4f,
				"yaw rate is P times the heading error");

		// Signs: a target rolled RIGHT of the aircraft asks for +roll rate.
		check(att.getRateTarget().z > 0.0f, "a target yawed right asks for +yaw");
		att.setEuler(-5.0f*D2R, 0.0f, 0.0f);
		att.update(level, 0.0025f);
		check(att.getRateTarget().x < 0.0f, "a target rolled left asks for -roll");
	}

	// -----------------------------------------------------------------------
	section("Limits");
	// -----------------------------------------------------------------------
	{
		Attitude att;
		const Quaternionf level = Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f);

		// A large heading error: P would ask for far more than the airframe can
		// stop from, so the stopping rate caps it.
		att.reset(level);
		att.setEuler(0.0f, 0.0f, 150.0f*D2R);
		att.update(level, 0.0025f);
		const float cap = Attitude::stoppingRate(150.0f*D2R, ATTITUDE_ACCEL_YAW_MAX);
		check(att.getLimits().accel_yaw, "a big heading error trips the accel limit");
		check(cap < ATTITUDE_P_YAW * 150.0f*D2R,
				"...which is less than P would have asked for");
		checkNear(att.getRateTarget().z, ATTITUDE_RATE_YAW_MAX, 1e-4f,
				"...though at THESE gains the rate limit is what finally binds");

		// The stopping cap in isolation, with the rate limit taken out of the
		// way. The two caps cross at exactly one error with the default gains
		// -- see the invariant below -- so nothing short of this separates them.
		Attitude uncapped;
		uncapped.reset(level);
		uncapped.setRateLimits(100.0f, 100.0f);
		uncapped.setEuler(0.0f, 0.0f, 150.0f*D2R);
		uncapped.update(level, 0.0025f);
		checkNear(uncapped.getRateTarget().z, cap, 1e-3f,
				"the capped demand IS the stopping rate");

		// WHICH cap binds first is a property of P, the rate limit and the
		// accel limit together, and it differs per axis. Pinned because
		// retuning any one of the three silently moves the crossover, and
		// because the two axes do NOT behave the same way here.
		//
		//   rate limit binds from   error > rate_max / P
		//   stopping cap binds from error > 2 * accel_max / P^2
		checkNear(ATTITUDE_RATE_YAW_MAX/ATTITUDE_P_YAW*R2D, 26.67f, 0.1f,
				"yaw: the rate limit binds from 26.7 deg of error");
		checkNear(2.0f*ATTITUDE_ACCEL_YAW_MAX
				/(ATTITUDE_P_YAW*ATTITUDE_P_YAW)*R2D, 26.67f, 0.1f,
				"...and the stopping cap from the same angle -- they meet");
		checkNear(ATTITUDE_RATE_RP_MAX/ATTITUDE_P_ROLL*R2D, 80.0f, 0.5f,
				"roll/pitch: the rate limit binds first, from 80 deg");
		checkNear(2.0f*ATTITUDE_ACCEL_RP_MAX
				/(ATTITUDE_P_ROLL*ATTITUDE_P_ROLL)*R2D, 108.6f, 0.5f,
				"...and the stopping cap only takes over past 108 deg");

		// Rate limit: raise the gains until P alone exceeds it.
		Attitude fast;
		fast.reset(level);
		fast.setGains(200.0f, 200.0f, 200.0f);
		fast.setAccelLimits(100000.0f, 100000.0f);   // take the stopping cap out
		fast.setEuler(60.0f*D2R, 0.0f, 0.0f);
		fast.update(level, 0.0025f);
		check(fast.getLimits().rate_rp, "the roll/pitch rate limit trips");
		checkNear(std::sqrt(fast.getRateTarget().x*fast.getRateTarget().x
		                  + fast.getRateTarget().y*fast.getRateTarget().y),
				ATTITUDE_RATE_RP_MAX, 1e-3f, "...and the pair is capped as a VECTOR");

		// The roll/pitch pair is limited as a vector, so a 45 deg demand does
		// not get sqrt(2) times the authority of a cardinal one.
		Attitude diag;
		diag.reset(level);
		diag.setGains(200.0f, 200.0f, 200.0f);
		diag.setAccelLimits(100000.0f, 100000.0f);
		diag.setEuler(45.0f*D2R, 45.0f*D2R, 0.0f);
		diag.update(level, 0.0025f);
		const Vector3f w = diag.getRateTarget();
		checkNear(std::sqrt(w.x*w.x + w.y*w.y), ATTITUDE_RATE_RP_MAX, 1e-3f,
				"a diagonal demand gets the SAME total as a cardinal one");
	}

	// -----------------------------------------------------------------------
	section("Yaw priority: heading is given up, tilt never is");
	// -----------------------------------------------------------------------
	{
		Attitude att;
		const Quaternionf level = Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f);
		att.reset(level);

		// Small tilt error: full heading authority.
		att.setEuler(10.0f*D2R, 0.0f, 20.0f*D2R);
		att.update(level, 0.0025f);
		checkNear(att.getYawPriority(), 1.0f, 1e-5f,
				"inside the priority angle, heading is fully corrected");
		check(!att.getLimits().yaw_priority, "...and nothing is reported");

		// Past twice the priority angle: heading abandoned entirely.
		att.setEuler(70.0f*D2R, 0.0f, 20.0f*D2R);
		att.update(level, 0.0025f);
		checkNear(att.getYawPriority(), 0.0f, 1e-5f,
				"past twice the angle, heading is given up completely");
		checkNear(att.getRateTarget().z, 0.0f, 1e-5f, "...so no yaw is commanded");
		check(att.getLimits().yaw_priority, "...and it is reported");
		check(std::fabs(att.getRateTarget().x) > 0.0f,
				"but the TILT correction is untouched");

		// Halfway: linear fade.
		att.setEuler(45.0f*D2R, 0.0f, 20.0f*D2R);
		att.update(level, 0.0025f);
		checkNear(att.getYawPriority(), 0.5f, 0.02f, "halfway is half authority");

		// The reported heading error is the REAL one, not the faded demand.
		checkNear(att.getYawError()*R2D, 20.0f, 1.0f,
				"getYawError() reports the angle that is actually wrong");
	}

	// -----------------------------------------------------------------------
	section("Feed-forward");
	// -----------------------------------------------------------------------
	{
		Attitude att;
		const Quaternionf level = Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f);
		att.reset(level);

		// Zero error, so every P term is zero and the output is the
		// feed-forward alone -- which is the proof it is added AFTER the
		// stopping-rate cap, since that cap is zero at zero error.
		att.setEuler(0.0f, 0.0f, 0.0f);
		att.setTargetAngularVelocity(Vector3f(0.0f, 0.0f, 1.5f));
		att.update(level, 0.0025f);
		checkNear(att.getRateTarget().z, 1.5f, 1e-4f,
				"at zero error the output IS the feed-forward");
		checkNear(att.getFeedForward().z, 1.5f, 1e-4f, "...and it is reported");

		// One cycle only: a stale feed-forward is a rate command nobody asked
		// for, so forgetting it must decay to zero rather than persist.
		att.update(level, 0.0025f);
		checkNear(att.getRateTarget().z, 0.0f, 1e-5f,
				"it is consumed after ONE cycle");
		checkNear(att.getFeedForward().z, 0.0f, 1e-6f, "...and reads zero after");

		// It adds to the correction rather than replacing it.
		att.setEuler(0.0f, 0.0f, 5.0f*D2R);
		att.setTargetAngularVelocity(Vector3f(0.0f, 0.0f, 1.0f));
		att.update(level, 0.0025f);
		checkNear(att.getRateTarget().z, ATTITUDE_P_YAW*5.0f*D2R + 1.0f, 1e-3f,
				"correction + feed-forward");

		// The rate limit still applies to the total.
		att.setEuler(0.0f, 0.0f, 0.0f);
		att.setTargetAngularVelocity(Vector3f(0.0f, 0.0f, 99.0f));
		att.update(level, 0.0025f);
		checkNear(att.getRateTarget().z, ATTITUDE_RATE_YAW_MAX, 1e-4f,
				"a huge feed-forward is still bounded by the rate limit");
		check(att.getLimits().rate_yaw, "...and reported");
	}

	// -----------------------------------------------------------------------
	section("Bad input is refused, and says so");
	// -----------------------------------------------------------------------
	{
		Attitude att;
		const Quaternionf level = Attitude::attitudeFromEuler(0.0f, 0.0f, 0.0f);
		att.reset(level);
		att.setEuler(5.0f*D2R, 0.0f, 0.0f);
		att.update(level, 0.0025f);

		const Vector3f good = att.getRateTarget();
		checkNear(att.getRejectedCount(), 0.0f, 0.5f,
				"nothing has been refused yet");

		const float nan = std::nanf("");

		att.update(Quaternionf(nan, 0.0f, 0.0f, 0.0f), 0.0025f);
		checkNear(att.getRateTarget().x, good.x, 1e-9f,
				"a non-finite attitude HOLDS the last demand");
		att.update(level, 0.0f);
		att.update(level, -0.01f);
		att.update(level, nan);
		checkNear(att.getRejectedCount(), 4.0f, 0.5f,
				"...and every refusal is counted");

		// Setters refuse too, and the previous target stands.
		const Vector3f before = att.getThrustVectorTarget();
		att.setThrustVectorHeading(Vector3f(0.0f, 0.0f, 0.0f), 0.0f);
		att.setThrustVectorHeading(Vector3f(nan, 0.0f, -1.0f), 0.0f);
		att.setEuler(nan, 0.0f, 0.0f);
		att.setTargetAngularVelocity(Vector3f(0.0f, 0.0f, nan));
		checkNear(angleBetween(att.getThrustVectorTarget(), before), 0.0f, 1e-4f,
				"a refused setter leaves the previous target standing");
		checkNear(att.getRejectedCount(), 8.0f, 0.5f,
				"refused setters are counted too");

		att.reset(level);
		checkNear(att.getRejectedCount(), 0.0f, 0.5f, "reset() clears the count");
	}

	// -----------------------------------------------------------------------
	section("Closed loop: it converges, and the thrust never wanders");
	// -----------------------------------------------------------------------
	{
		// Flown with a perfect rate loop (the aircraft turns at exactly the
		// commanded rate), which isolates THIS loop's behaviour.
		const float dt = 0.0025f;

		// 1. A large upset converges to the target.
		{
			Attitude att;
			Quaternionf q = Attitude::attitudeFromEuler(60.0f*D2R, -40.0f*D2R,
					10.0f*D2R);
			att.reset(q);
			att.setEuler(0.0f, 0.0f, 0.0f);
			for (int i = 0; i < 2000; i++) {          // 5 s
				att.update(q, dt);
				q = integrate(q, att.getRateTarget(), dt);
			}
			att.update(q, dt);
			checkNear(att.getTiltError()*R2D, 0.0f, 0.2f,
					"a 70 deg upset converges (tilt, deg)");
			checkNear(att.getYawError()*R2D, 0.0f, 0.2f, "...and the heading too");
		}

		// 2. THE one that matters: sweep the heading target away from the
		//    aircraft at the rate limit with NO feed-forward, so the target
		//    genuinely runs ahead by rate/P = 120/4.5 = 26.7 deg -- and the
		//    thrust vector, held due EAST in NED, must not move at all.
		//
		//    A heading error is a rotation ABOUT the thrust axis, and a rotation
		//    about an axis cannot move that axis. This is the test that says so.
		{
			Attitude att;
			Quaternionf q = Attitude::attitudeFromEuler(0.0f, 0.0f, 90.0f*D2R);
			att.reset(q);

			const Vector3f t_east = leanToward(10.0f*D2R, 90.0f);
			float heading = 90.0f*D2R;
			float worst = 0.0f, worst_lead = 0.0f;

			for (int i = 0; i < 1600; i++) {          // 4 s of full yaw stick
				const float measured = Attitude::headingFromAttitude(q);

				// Mode layer: sweep the heading target, clamped so it cannot run
				// away from the aircraft (see the header).
				heading += ATTITUDE_RATE_YAW_MAX * dt;
				if (Attitude::wrapPi(heading - measured) > 40.0f*D2R) {
					heading = measured + 40.0f*D2R;
				}

				att.setThrustVectorHeading(t_east, heading);
				att.update(q, dt);
				q = integrate(q, att.getRateTarget(), dt);

				if (i > 600) {                        // past the initial lean settle
					const float e = angleBetween(
							Attitude::thrustVectorOf(q), t_east);
					if (e > worst) worst = e;
					const float l = std::fabs(Attitude::wrapPi(heading
							- Attitude::headingFromAttitude(q))) * R2D;
					if (l > worst_lead) worst_lead = l;
				}
			}
			checkNear(worst_lead, ATTITUDE_RATE_YAW_MAX/ATTITUDE_P_YAW*R2D, 2.0f,
					"the heading target leads the aircraft by rate/P (deg)");
			checkNear(worst, 0.0f, 0.05f,
					"...yet the achieved thrust stayed EAST throughout (deg)");
		}

		// 2b. The same sweep WITH the feed-forward. Its whole job is the nose:
		//     the lead above should collapse, while the thrust -- which was never
		//     at risk -- stays exactly where it was.
		{
			Attitude att;
			Quaternionf q = Attitude::attitudeFromEuler(0.0f, 0.0f, 90.0f*D2R);
			att.reset(q);

			const Vector3f t_east = leanToward(10.0f*D2R, 90.0f);
			float heading = 90.0f*D2R;
			float worst = 0.0f, worst_lead = 0.0f;

			for (int i = 0; i < 1600; i++) {
				const float measured = Attitude::headingFromAttitude(q);
				heading += ATTITUDE_RATE_YAW_MAX * dt;
				if (Attitude::wrapPi(heading - measured) > 40.0f*D2R) {
					heading = measured + 40.0f*D2R;
				}

				att.setThrustVectorHeading(t_east, heading);
				att.setTargetAngularVelocity(
						Vector3f(0.0f, 0.0f, ATTITUDE_RATE_YAW_MAX));
				att.update(q, dt);
				q = integrate(q, att.getRateTarget(), dt);

				if (i > 600) {
					const float e = angleBetween(
							Attitude::thrustVectorOf(q), t_east);
					if (e > worst) worst = e;
					const float l = std::fabs(Attitude::wrapPi(heading
							- Attitude::headingFromAttitude(q))) * R2D;
					if (l > worst_lead) worst_lead = l;
				}
			}
			check(worst_lead < 2.0f,
					"the feed-forward removes the heading lag from the NOSE");
			checkNear(worst, 0.0f, 0.05f,
					"...and the thrust is unchanged -- it was never at risk (deg)");
		}

		// 3. Control: the same sweep with the double-rotation bug injected --
		//    a lean solved at the MEASURED heading but applied at the TARGET
		//    heading. If this does not fail, test 2 proves nothing.
		{
			Attitude att;
			Quaternionf q = Attitude::attitudeFromEuler(0.0f, 0.0f, 90.0f*D2R);
			att.reset(q);

			const Vector3f t_east = leanToward(10.0f*D2R, 90.0f);
			float heading = 90.0f*D2R;
			float worst = 0.0f;

			for (int i = 0; i < 1200; i++) {
				const float measured = Attitude::headingFromAttitude(q);
				heading += ATTITUDE_RATE_YAW_MAX * dt;
				if (Attitude::wrapPi(heading - measured) > 40.0f*D2R) {
					heading = measured + 40.0f*D2R;
				}

				// The bug: solve the lean at `measured`, then hand the
				// resulting angles to setEuler() at `heading`, which rotates
				// them a second time.
				const Quaternionf solved =
						Attitude::attitudeFromThrustVector(t_east, measured);
				const Vector3f rv = Attitude::rotationVector(solved);
				att.setEuler(rv.x, rv.y, heading);
				att.setTargetAngularVelocity(
						Vector3f(0.0f, 0.0f, ATTITUDE_RATE_YAW_MAX));
				att.update(q, dt);
				q = integrate(q, att.getRateTarget(), dt);

				if (i > 200) {
					const float e = angleBetween(
							Attitude::thrustVectorOf(q), t_east);
					if (e > worst) worst = e;
				}
			}
			check(worst > 5.0f,
					"the double-rotation bug IS detected by this harness (deg)");
		}
	}

	return testReport("Controllers");
}
