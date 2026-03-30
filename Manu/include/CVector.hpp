#pragma once

#include <array>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

template<typename T, std::size_t N>
/**
 * @class CVecteur
 * @brief Classe template représentant un vecteur de taille N
 *
 * @tparam T Type des éléments
 * @tparam N Taille du vecteur
 */
class CVecteur{
private:
    std::array<T, N> data_;

public:
    // ===== Constructeurs =====
    CVecteur(){
        data_.fill(T(0));
    }

    CVecteur(const std::array<T, N>& data) : data_(data) {}

    // ===== Accès =====
    T& operator[](std::size_t i){
        if(i >= N) throw std::out_of_range("Index hors limites");
        return data_[i];
    }

    const T& operator[](std::size_t i) const{
        if(i >= N) throw std::out_of_range("Index hors limites");
        return data_[i];
    }

    // ===== Addition =====
    CVecteur operator+(const CVecteur& other) const{
        CVecteur result;
        for(std::size_t i = 0; i < N; i++)
            result[i] = data_[i] + other[i];
        return result;
    }

    // ===== Soustraction =====
    CVecteur operator-(const CVecteur& other) const{
        CVecteur result;
        for(std::size_t i = 0; i < N; i++)
            result[i] = data_[i] - other[i];
        return result;
    }

    // ===== Multiplication scalaire =====
    CVecteur operator*(T scalar) const{
        CVecteur result;
        for(std::size_t i = 0; i < N; i++)
            result[i] = data_[i] * scalar;
        return result;
    }

    // ===== Produit scalaire =====
    T dot(const CVecteur& other) const{
        T result = T(0);
        for(std::size_t i = 0; i < N; i++)
            result += data_[i] * other[i];
        return result;
    }

    // ===== Norme =====
    T norm() const{
        return std::sqrt(this->dot(*this));
    }

    // ===== Conversion Eigen =====
    Eigen::Matrix<T, N, 1> toEigen() const{
        return Eigen::Map<const Eigen::Matrix<T, N, 1>>(data_.data());
    }

    // ===== Affichage =====
    friend std::ostream& operator<<(std::ostream& os, const CVecteur& v){
        os << "[";
        for(std::size_t i = 0; i < N; i++)
        {
            os << v[i];
            if(i < N - 1) os << ", ";
        }
        os << "]";
        return os;
    }
};