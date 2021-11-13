#ifndef _FeedForward_
#define _FeedForward_

#include <Arduino.h>

class FeedForward
{
    private:

        float previous_target_state_value_ = 0;

        float feed_forward_output_ = 0;

		float polynomial_coefficients_positive_value_positive_derivative_[5] = {0, 0, 0, 0, 0};
		float polynomial_coefficients_positive_value_negative_derivative_[5] = {0, 0, 0, 0, 0};
		float polynomial_coefficients_negative_value_positive_derivative_[5] = {0, 0, 0, 0, 0};
		float polynomial_coefficients_negative_value_negative_derivative_[5] = {0, 0, 0, 0, 0};

		inline float compute_polynomial(float val, float polynomial_coefficients[5])
		{
			return polynomial_coefficients[0] +
			polynomial_coefficients[1] * val +
			polynomial_coefficients[2] * pow(val, 2) +
			polynomial_coefficients[3] * pow(val, 3) +
			polynomial_coefficients[4] * pow(val, 4);
		}

    public:

        inline void setTargetStateValue_FF(float target_state_value)
		{
			if (signbit(target_state_value))
			{
				// if target state value is negative
				
				if (target_state_value > previous_target_state_value_)
					// if time derivative of target state is positive
					feed_forward_output_ = compute_polynomial(target_state_value, polynomial_coefficients_negative_value_positive_derivative_);

				else
					// if time derivative of target state is negative
					feed_forward_output_ = compute_polynomial(target_state_value, polynomial_coefficients_negative_value_negative_derivative_);
			}
			else
			{
				// if target state value is positive

				if (target_state_value > previous_target_state_value_)
					// if time derivative of target state is positive
					feed_forward_output_ = compute_polynomial(target_state_value, polynomial_coefficients_positive_value_positive_derivative_);

				else
					// if time derivative of target state is negative
					feed_forward_output_ = compute_polynomial(target_state_value, polynomial_coefficients_positive_value_negative_derivative_);
			}

			previous_target_state_value_ = target_state_value;
		}
		
		inline float getFeedForwardControllerOutput() __attribute__((always_inline))
		{
			return feed_forward_output_;
		}

		void set_polynomial_coefficients_positive_value_positive_acceleration(
			float coefficient_0,
			float coefficient_1,
			float coefficient_2,
			float coefficient_3,
			float coefficient_4
		) {
			polynomial_coefficients_positive_value_positive_derivative_[0] = coefficient_0;
			polynomial_coefficients_positive_value_positive_derivative_[1] = coefficient_1;
			polynomial_coefficients_positive_value_positive_derivative_[2] = coefficient_2;
			polynomial_coefficients_positive_value_positive_derivative_[3] = coefficient_3;
			polynomial_coefficients_positive_value_positive_derivative_[4] = coefficient_4;
		}

		void set_polynomial_coefficients_positive_value_negative_acceleration(
			float coefficient_0,
			float coefficient_1,
			float coefficient_2,
			float coefficient_3,
			float coefficient_4
		) {
			polynomial_coefficients_positive_value_negative_derivative_[0] = coefficient_0;
			polynomial_coefficients_positive_value_negative_derivative_[1] = coefficient_1;
			polynomial_coefficients_positive_value_negative_derivative_[2] = coefficient_2;
			polynomial_coefficients_positive_value_negative_derivative_[3] = coefficient_3;
			polynomial_coefficients_positive_value_negative_derivative_[4] = coefficient_4;
		}

		void set_polynomial_coefficients_negative_value_positive_acceleration(
			float coefficient_0,
			float coefficient_1,
			float coefficient_2,
			float coefficient_3,
			float coefficient_4
		) {
			polynomial_coefficients_negative_value_positive_derivative_[0] = coefficient_0;
			polynomial_coefficients_negative_value_positive_derivative_[1] = coefficient_1;
			polynomial_coefficients_negative_value_positive_derivative_[2] = coefficient_2;
			polynomial_coefficients_negative_value_positive_derivative_[3] = coefficient_3;
			polynomial_coefficients_negative_value_positive_derivative_[4] = coefficient_4;
		}

		void set_polynomial_coefficients_negative_value_negative_acceleration(
			float coefficient_0,
			float coefficient_1,
			float coefficient_2,
			float coefficient_3,
			float coefficient_4
		) {
			polynomial_coefficients_negative_value_negative_derivative_[0] = coefficient_0;
			polynomial_coefficients_negative_value_negative_derivative_[1] = coefficient_1;
			polynomial_coefficients_negative_value_negative_derivative_[2] = coefficient_2;
			polynomial_coefficients_negative_value_negative_derivative_[3] = coefficient_3;
			polynomial_coefficients_negative_value_negative_derivative_[4] = coefficient_4;
		}
};

#endif