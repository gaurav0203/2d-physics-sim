/*
basic 2d ball physics sim
everything is combined in the ball class only
*/

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <cmath>   // for std:: abs to use with float
#include <cstdlib> // for rand()
#include <ctime>   // for time()
#include <sstream> // for ostringstream

const float GRAVITY = 1500.0f;
const float DAMPING = 0.85f;

class Ball
{
private:
    sf::CircleShape shape;
    sf::Vector2f velocity;

    float mass = 1.0f;

public:
    Ball(float x, float y, float radius)
    {

        shape.setPosition(x, y);
        shape.setRadius(radius);

        shape.setFillColor(sf::Color(std::rand() % 256, std::rand() % 256, std::rand() % 256));
        shape.setOrigin(radius, radius);

        float vx = static_cast<float>((std::rand() % 600) - 300);
        float vy = static_cast<float>((std::rand() % 600) - 300);
        velocity = {vx, vy};

        mass = radius * radius;
    }

    void update(const sf::RenderWindow &window, float dt)
    {

        velocity.y += GRAVITY * dt;

        shape.move(velocity * dt);

        sf::Vector2f pos = shape.getPosition();
        float r = shape.getRadius();
        sf::Vector2u win_size = window.getSize();

        // left right walls collision
        if (pos.x - r < 0)
        {
            shape.setPosition(r, pos.y);
            velocity.x = std::abs(velocity.x);
        }
        else if (pos.x + r > win_size.x)
        {
            shape.setPosition(win_size.x - r, pos.y);
            velocity.x = -std::abs(velocity.x);
        }

        // ceiling collision
        if (pos.y - r < 0)
        {
            shape.setPosition(pos.x, r);
            velocity.y = std::abs(velocity.y);
        }
        // floor collision
        else if (pos.y + r > win_size.y)
        {
            shape.setPosition(pos.x, win_size.y - r);
            velocity.y = -std::abs(velocity.y) * DAMPING;

            if (std::abs(velocity.y) < 30.0f)
            {
                velocity.y = 0;
            }
        }
    }

    sf::Vector2f getPosition()
    {
        return shape.getPosition();
    }

    void setPosition(sf::Vector2f pos)
    {
        shape.setPosition(pos);
    }

    sf::Vector2f getVelocity()
    {
        return velocity;
    }

    void setVelocity(sf::Vector2f vel)
    {
        velocity = vel;
    }

    float getRadius()
    {
        return shape.getRadius();
    }

    float getMass()
    {
        return mass;
    }

    void render(sf::RenderWindow &window) const
    {
        window.draw(shape);
    }
};

void handleCollision(Ball &ballA, Ball &ballB)
{
    sf::Vector2f posA = ballA.getPosition();
    sf::Vector2f posB = ballB.getPosition();

    sf::Vector2f delta = posA - posB; // vector pointing B -> A

    float distSq = delta.x * delta.x + delta.y * delta.y;
    float minDist = ballA.getRadius() + ballB.getRadius();

    // Collision!
    if (distSq < minDist * minDist)
    {
        float dist = std::sqrt(distSq);

        if (dist == 0.0f)
            return; // prevent divide by zero if spawning at same place

        float overlap = minDist - dist;

        // Static resolution - to un-overlap balls

        sf::Vector2f n = delta / dist;

        float mass_sum = ballA.getMass() + ballB.getMass();

        ballA.setPosition(posA + n * overlap * (ballB.getMass() / mass_sum));
        ballB.setPosition(posB - n * overlap * (ballA.getMass() / mass_sum));

        // Dynamic resolution - to determine new velocity

        sf::Vector2f relVel = ballA.getVelocity() - ballB.getVelocity(); // vel of a wrt b
        float velAlongNormal = relVel.x * n.x + relVel.y * n.y;          // component vel of a wrt b along the normal

        if (velAlongNormal > 0)
            return; // means ball A is moving away from ball B

        /*
            dont want to deal with time so we use impulse instead of force
            J = del p = m. del v;
            F = ma;
            F = J/t;
            J = j.n since no friction impulse only along n
            --------------

            del v = J/m
            v_new - v_old = J/m
            v_new = v_old + J/m

            applying this to our case

            (dot n to get impulse along normal)

            v_new_a = v_old_a + j.n/m_a...1
            v_new_b = v_old_b - j.n/m_b...2

            restitution formula
            (v_new_a - v_new_b).n = -e.(v_old_a - v_old_b).n
            v_new_rel.n = -e.v_old_rel.n

            subtract 1-2
            v_new_rel = v_old_rel + j.n(1/m_a + 1/m_b)

            to get rel vel only along normal mul by n also n.n = 1

            v_new_rel.n = v_old_rel.n + j(1/m_a + 1/m_b)

            -e.v_old_rel.n = v_old_rel.n + j(1/m_a + 1/m_b)

            -(1+e)v_old_rel.n = j(1/m_a + 1/m_b)

            j = -(1+e)v_old_rel.n/(1/m_a + 1/m_b)
        */

        float j = -(1 + 0.8) * velAlongNormal;
        j /= (1.0f / ballA.getMass()) + (1.0f / ballB.getMass());

        sf::Vector2f impulse = j * n;
        ballA.setVelocity(ballA.getVelocity() + impulse / ballA.getMass());
        ballB.setVelocity(ballB.getVelocity() - impulse / ballB.getMass());
    }
}

int main()
{
    const int WINDOW_WIDTH = 600;
    const int WINDOW_HEIGHT = 600;
    const int FRAME_RATE = 60;
    const int SUB_STEP = 8; // number of times updating physics for each ball, every frame
    const std::string WINDOW_TITLE = "Ball Sim";
    std::ostringstream ss;

    // generating seed according to current time. the seed helps mixing up random so it is different every start of code
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), WINDOW_TITLE);
    window.setFramerateLimit(FRAME_RATE);
    // window.setFramerateLimit(0); // unlimited fps to for stress test

    Ball ball(10, 10, 5);

    std::vector<Ball> objects;
    objects.push_back(ball);

    sf::Clock clock;
    sf::Clock fpsClock;

    while (window.isOpen())
    {
        sf::Event event;

        float dt = clock.restart().asSeconds();
        if (dt > 0.1f)
        {
            dt = 0.1f; // lag spike protection
        }
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed)
            {
                // objects.clear();
                float radius = static_cast<float>((std::rand() % 20) + 10);
                objects.emplace_back(event.mouseButton.x, event.mouseButton.y, radius);
            }
        }

        window.clear();

        for (auto &obj : objects)
        {
            obj.update(window, dt);
        }

        for (int k = 0; k < SUB_STEP; k++)
        {
            for (int i = 0; i < objects.size(); i++)
            {
                for (int j = i + 1; j < objects.size(); j++)
                {
                    handleCollision(objects[i], objects[j]);
                }
            }
        }

        for (auto &obj : objects)
        {
            obj.render(window);
        }

        window.display();

        if (fpsClock.getElapsedTime().asSeconds() > 0.1f)
        {
            float fps = 1.0 / dt;

            ss.str("");
            ss.clear();

            ss << "Balls: " << objects.size() << " | FPS: " << static_cast<int>(fps);
            window.setTitle(ss.str());

            fpsClock.restart();
        }
    }

    return 0;
}
