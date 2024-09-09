package org.firstinspires.ftc.teamcode.utils;

public interface Utils {
  static <T> boolean every(T[] arr, Predicate<T> predicate) {
    for (T t : arr) {
      if (!predicate.test(t)) {
        return false;
      }
    }
    return true;
  }
}

//All Utils code was made by Orville, There Undocumented and I don't know what they do