import itertools

# sensor model for true and false positives

starting_prob = 0.5
prob_sense_emp_given_occ = 0.2
prob_sense_emp_given_emp = 0.9
prob_sense_occ_given_occ_rate = 0.1
prob_sense_occ_given_occ_offset = 0.3
prob_sense_occ_given_emp_rate = -0.1
prob_sense_occ_given_emp_offset = 0.5


def dynamic_sense_occ_given_occ(readings):

  # Need to figure out how to make customizable, but essentially
  # one lidar point is not a lot of confidence, 3 or more and we are starting to feel confident
  # This is similar to the RELU function with some manual offset and sloping
  # y = 0.05 * readings + 0.5 - line formula
  # 0.99 is max confidence of obstacle we will allow
  return min(0.99, (readings * prob_sense_occ_given_occ_rate) + prob_sense_occ_given_occ_offset)


def dynamic_sense_occ_given_emp(readings):

  # Opposite of previous function. We lose confidence
  # that a cell is empty occupied as the number of readings increases
  # y = -0.1 * readings + 0.3
  return max(0.01, (readings * prob_sense_occ_given_emp_rate) + prob_sense_occ_given_emp_offset)


def probability_update(readings, current_prob_occ):

  # Determine probabilities to use based on number lidar points that hit the cell
  prob_given_occ_to_use = 0
  prob_given_emp_to_use = 0

  # If no lidar points
  if (readings == 0):
    prob_given_occ_to_use = prob_sense_emp_given_occ
    prob_given_emp_to_use = prob_sense_emp_given_emp
  else:
    # If we had positive readings, dynamically calculate the confidences to use
    prob_given_occ_to_use = dynamic_sense_occ_given_occ(readings)
    prob_given_emp_to_use = dynamic_sense_occ_given_emp(readings)

  belief_occ = prob_given_occ_to_use * current_prob_occ
  belief_emp = prob_given_emp_to_use * (1 - current_prob_occ)

  # print("Belief occ: " + str(belief_occ))
  # print("Belief emp: " + str(belief_emp))

  normalizer = 1 / (belief_occ + belief_emp)

  return belief_occ * normalizer


# Calculate the update probability given the reading and current probability of occupied
# def binary_probability_update(reading, current_prob_occ):

#   prob_given_occ_to_use = prob_sense_occ_given_occ if reading else prob_sense_emp_given_occ
#   prob_given_emp_to_use = prob_sense_occ_given_emp if reading else prob_sense_emp_given_emp

#   belief_occ = prob_given_occ_to_use * current_prob_occ
#   belief_emp = prob_given_emp_to_use * (1 - current_prob_occ)

#   normalizer = 1 / (belief_occ + belief_emp)

#   return belief_occ * normalizer

# Want to results the above function gives for different branches


def main():

  record = dict()

  l = itertools.product([0,1,2,3,4,5,6,7,8,9], repeat=1) # Number of hits

  for read in l:
    current_prob = starting_prob
    for p in read:
      current_prob = probability_update(p, current_prob)

    print(str(read) + str(current_prob))


  # for i in range(7):
  #   read_seq_set = itertools.product([True, False], repeat=i)

  #   for read_seq in read_seq_set:
  #     # Starting new sequence, everything equally likely
  #     current_prob = 0.5
  #     for reading in read_seq:
  #       current_prob = binary_probability_update(reading, current_prob)

  #     record[read_seq] = current_prob

  # for r in record.keys():
  #   print(str(r) + ":" + str(record[r]))


if __name__ == '__main__':
  main()
